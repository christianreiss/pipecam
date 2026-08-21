// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * pipecam - V4L2 driver for the "Look Kellyop lem01camera" USB pipe/endoscope
 *           camera (USB ID 3456:4321).
 *
 * The device is NOT a UVC device: both of its interfaces are vendor-specific
 * (bInterfaceClass 0xFF / bInterfaceSubClass 0xF0) and it carries no UVC class
 * descriptors at all, so uvcvideo cannot bind it.  Interface 0 is an Apple MFi
 * "iAP Interface"; interface 1 exposes the External Accessory protocol
 * "com.linkback.protocol" and carries the video stream.
 *
 * Wire format (established by capture and analysis of the live device):
 *
 *   - Interface 1 altsetting 1 exposes bulk IN endpoint 0x82.  Selecting that
 *     altsetting starts the stream; selecting altsetting 0 (which has no IN
 *     endpoint) stops it.  There is no vendor "start" command - the device
 *     streams unprompted as soon as the altsetting is selected.
 *
 *   - The stream consists of logical packets aligned at 512-byte offsets
 *     within each bulk transfer.  Every packet carries its own UVC-style
 *     payload header; the final packet in a transfer is frequently short:
 *
 *         byte 0 : bHeaderLength (always 12 in practice)
 *         byte 1 : bmHeaderInfo - bit0 FID, bit1 EOF, bit2 PTS,
 *                                 bit3 SCR, bit6 ERR, bit7 EOH
 *         byte 2..5  : PTS
 *         byte 6..11 : SCR
 *
 *     Observed header info bytes are 0x8C/0x8D (EOH|SCR|PTS with the frame ID
 *     toggling) and 0x8E/0x8F for the final packet of each frame.
 *
 *   - A full packet therefore carries 500 payload bytes.  Short final packets
 *     make the packet count variable, but one frame always contains exactly
 *     405000 payload bytes.
 *
 *   - Frame geometry, recovered by autocorrelation of the payload stream
 *     (strong peak at lag 900 with harmonics at 1800/2700/3600/4500) and
 *     confirmed visually: 450x450, 900 bytes per line, 405000 bytes per frame.
 *
 *   - Pixel format is YUYV 4:2:2: luma occupies the even byte offsets (wide
 *     distribution) while chroma occupies the odd offsets and is tightly
 *     centred on 0x80.
 *
 *   - Measured frame rate is approximately 20-27 fps, controlled by the
 *     sensor's auto-exposure.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/ktime.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define PIPECAM_DRIVER		"pipecam"
#define PIPECAM_CARD		"Look Kellyop lem01camera"

#define PIPECAM_VID		0x3456
#define PIPECAM_PID		0x4321

/* Video interface and its altsettings. */
#define PIPECAM_IFNUM		1
#define PIPECAM_ALT_IDLE	0
#define PIPECAM_ALT_STREAM	1
#define PIPECAM_EP_IN		0x82

/* Frame geometry. */
#define PIPECAM_WIDTH		450U
#define PIPECAM_HEIGHT		450U
#define PIPECAM_BPL		(PIPECAM_WIDTH * 2)		/* 900 */
#define PIPECAM_IMGSIZE		(PIPECAM_BPL * PIPECAM_HEIGHT)	/* 405000 */
/* Payload framing. */
#define PIPECAM_USB_PKT		512
#define PIPECAM_HDR_MIN		2

/* URB pool.  Buffers are a multiple of the logical 512-byte packet span.  A
 * USB short packet ends its bulk transfer and is handled as the final,
 * shorter logical packet by pipecam_process().
 */
#define PIPECAM_NURBS		16
#define PIPECAM_URB_SIZE	(PIPECAM_USB_PKT * 64)		/* 32 KiB */
#define PIPECAM_MAX_URB_ERRORS	256

struct pipecam_buf {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
};

enum pipecam_stream_state {
	PIPECAM_STOPPED,
	PIPECAM_STARTING,
	PIPECAM_RUNNING,
	PIPECAM_STOPPING,
	PIPECAM_FAILED,
};

struct pipecam {
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	struct vb2_queue	queue;

	struct mutex		lock;	/* serialises ioctls / vb2 queue ops */
	spinlock_t		qlock;	/* queue, assembly and URB error state */

	struct usb_device	*udev;
	struct usb_interface	*intf;
	bool			present;	/* false once disconnected */

	struct urb		*urb[PIPECAM_NURBS];
	unsigned int		urb_errors;
	enum pipecam_stream_state stream_state;
	struct list_head	buf_list;

	/* Frame assembly state, touched from URB completion under qlock. */
	struct pipecam_buf	*cur;
	void			*cur_vaddr;
	unsigned int		cur_off;
	unsigned int		frame_bytes;
	bool			fid_valid;
	bool			frame_synced;
	bool			frame_bad;
	u8			cur_fid;
	u32			sequence;
};

static inline struct pipecam_buf *to_pipecam_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct pipecam_buf, vb);
}

/* ------------------------------------------------------------------ */
/* Frame assembly                                                      */
/* ------------------------------------------------------------------ */

/* Called with qlock held. */
static void pipecam_frame_end(struct pipecam *dev)
{
	struct pipecam_buf *buf = dev->cur;
	u64 now;
	u32 sequence;
	unsigned int bytes = dev->frame_bytes;
	bool bad = dev->frame_bad;
	bool synced = dev->frame_synced;

	dev->frame_bad = false;
	dev->frame_bytes = 0;

	/* After initial synchronization, count every frame boundary, including
	 * frames dropped because userspace supplied no buffer or because the
	 * payload was malformed.  This makes gaps in v4l2_buffer.sequence
	 * meaningful instead of hiding drops.
	 */
	now = ktime_get_ns();
	dev->frame_synced = true;

	/* The first boundary after streamon may end a frame that began before
	 * every URB was submitted.  Use it for synchronization, but do not report
	 * that unavoidable partial frame as a userspace-visible drop.
	 */
	if (!synced && (bad || !buf || bytes != PIPECAM_IMGSIZE ||
			dev->cur_off != PIPECAM_IMGSIZE)) {
		if (buf)
			dev->cur_off = 0;
		return;
	}
	sequence = dev->sequence++;

	if (!buf)
		return;

	/* Only hand over frames that are exactly the expected size.  Partial
	 * frames happen after streamon (we may join the stream mid-frame) and
	 * whenever a URB is dropped; recycle the buffer instead of delivering
	 * a torn image.
	 */
	if (bad || bytes != PIPECAM_IMGSIZE ||
	    dev->cur_off != PIPECAM_IMGSIZE) {
		dev->cur_off = 0;
		return;
	}

	buf->vb.vb2_buf.timestamp = now;
	buf->vb.sequence = sequence;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, PIPECAM_IMGSIZE);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	dev->cur = NULL;
	dev->cur_vaddr = NULL;
	dev->cur_off = 0;
}

/* Called with qlock held. */
static void pipecam_append(struct pipecam *dev, const u8 *p, int n)
{
	unsigned int room;

	if (n <= 0)
		return;

	if (dev->frame_bytes >= PIPECAM_IMGSIZE ||
	    n > PIPECAM_IMGSIZE - dev->frame_bytes) {
		dev->frame_bad = true;
		dev->frame_bytes = PIPECAM_IMGSIZE + 1;
		return;
	}
	dev->frame_bytes += n;

	if (!dev->cur) {
		if (list_empty(&dev->buf_list))
			return;		/* no buffer available - drop */
		dev->cur = list_first_entry(&dev->buf_list,
					    struct pipecam_buf, list);
		list_del(&dev->cur->list);
		dev->cur_vaddr = vb2_plane_vaddr(&dev->cur->vb.vb2_buf, 0);
		dev->cur_off = 0;
		if (!dev->cur_vaddr) {
			vb2_buffer_done(&dev->cur->vb.vb2_buf,
					VB2_BUF_STATE_ERROR);
			dev->cur = NULL;
			return;
		}
	}

	room = PIPECAM_IMGSIZE - dev->cur_off;
	if (n > room) {
		/* The buffer was acquired after this logical frame began. */
		dev->frame_bad = true;
		return;
	}

	memcpy(dev->cur_vaddr + dev->cur_off, p, n);
	dev->cur_off += n;
}

/*
 * Walk one URB buffer.  Each logical packet at a 512-byte offset carries its
 * own payload header, so the header must be stripped per packet rather than
 * once per transfer - doing the latter yields a subtly sheared image.
 */
static void pipecam_process(struct pipecam *dev, const u8 *data, int len)
{
	unsigned long flags;
	int off;

	spin_lock_irqsave(&dev->qlock, flags);
	if (dev->stream_state != PIPECAM_RUNNING)
		goto unlock;

	for (off = 0; off < len; off += PIPECAM_USB_PKT) {
		const u8 *pkt = data + off;
		int clen = min_t(int, PIPECAM_USB_PKT, len - off);
		u8 hlen, info, fid;
		bool eof;

		if (clen < PIPECAM_HDR_MIN) {
			dev->frame_bad = true;
			break;
		}

		hlen = pkt[0];
		info = pkt[1];

		/* Ignore malformed packets rather than resynchronising blindly. */
		if (hlen < PIPECAM_HDR_MIN || hlen > clen) {
			dev->frame_bad = true;
			continue;
		}

		fid = info & 0x01;
		eof = !!(info & 0x02);

		if (dev->fid_valid && fid != dev->cur_fid)
			pipecam_frame_end(dev);

		dev->cur_fid = fid;
		dev->fid_valid = true;

		/* UVC-style bit 6 marks the current payload as erroneous.  Do not
		 * allow an exact byte count to turn such a frame into valid output.
		 */
		if (info & BIT(6))
			dev->frame_bad = true;
		else
			pipecam_append(dev, pkt + hlen, clen - hlen);

		if (eof) {
			pipecam_frame_end(dev);
			dev->fid_valid = false;
		}
	}

unlock:
	spin_unlock_irqrestore(&dev->qlock, flags);
}

static bool pipecam_fail_stream(struct pipecam *dev)
{
	unsigned long flags;
	bool first, notify;

	spin_lock_irqsave(&dev->qlock, flags);
	first = dev->stream_state == PIPECAM_STARTING ||
		dev->stream_state == PIPECAM_RUNNING;
	notify = dev->stream_state == PIPECAM_RUNNING;
	if (first)
		dev->stream_state = PIPECAM_FAILED;
	spin_unlock_irqrestore(&dev->qlock, flags);

	if (notify)
		vb2_queue_error(&dev->queue);
	return first;
}

static void pipecam_urb_complete(struct urb *urb)
{
	struct pipecam *dev = urb->context;
	unsigned long flags;
	bool active, process, stop, first, notify;
	int ret;

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ECONNRESET:
	case -ESHUTDOWN:
		return;			/* unlinked - we are stopping */
	default:
		/* Give up on an endpoint that only produces errors rather than
		 * resubmitting forever in completion context.  A stall needs
		 * usb_clear_halt(), which may sleep and so cannot be done here;
		 * the user can recover by restarting the stream.
		 */
		spin_lock_irqsave(&dev->qlock, flags);
		active = dev->stream_state == PIPECAM_STARTING ||
			 dev->stream_state == PIPECAM_RUNNING;
		stop = !active;
		first = false;
		notify = false;
		if (active) {
			/* Error completions may still contain partial data.  Discard it
			 * and poison the logical frame independently of buffer ownership.
			 */
			dev->frame_bad = true;
			if (urb->status == -EPIPE ||
			    ++dev->urb_errors >= PIPECAM_MAX_URB_ERRORS) {
				notify = dev->stream_state == PIPECAM_RUNNING;
				dev->stream_state = PIPECAM_FAILED;
				stop = true;
				first = true;
			}
		}
		spin_unlock_irqrestore(&dev->qlock, flags);

		if (first) {
			/* Report the failure to userspace instead of stalling
			 * silently: DQBUF returns EIO and the application can
			 * recover by restarting the stream.
			 */
			dev_err(&dev->intf->dev,
				"endpoint failing (status %d) - stopping stream; restart capture to recover\n",
				urb->status);
			if (notify)
				vb2_queue_error(&dev->queue);
		}
		if (stop)
			return;
		dev_dbg(&dev->intf->dev, "URB status %d\n", urb->status);
		goto resubmit;
	}

	spin_lock_irqsave(&dev->qlock, flags);
	active = dev->stream_state == PIPECAM_STARTING ||
		 dev->stream_state == PIPECAM_RUNNING;
	process = dev->stream_state == PIPECAM_RUNNING;
	if (active)
		dev->urb_errors = 0;
	spin_unlock_irqrestore(&dev->qlock, flags);
	if (!active)
		return;

	if (process && urb->actual_length > 0)
		pipecam_process(dev, urb->transfer_buffer, urb->actual_length);

resubmit:
	spin_lock_irqsave(&dev->qlock, flags);
	active = dev->stream_state == PIPECAM_STARTING ||
		 dev->stream_state == PIPECAM_RUNNING;
	spin_unlock_irqrestore(&dev->qlock, flags);
	if (!active)
		return;

	ret = usb_submit_urb(urb, GFP_ATOMIC);
	/* -EPERM/-ENODEV/-ESHUTDOWN/-ENOENT are the normal teardown races. */
	if (ret && ret != -EPERM && ret != -ENODEV &&
	    ret != -ESHUTDOWN && ret != -ENOENT &&
	    pipecam_fail_stream(dev))
		dev_err(&dev->intf->dev,
			"URB resubmit failed: %d - stopping stream\n", ret);
}

/* ------------------------------------------------------------------ */
/* URB pool                                                            */
/* ------------------------------------------------------------------ */

static void pipecam_free_urbs(struct pipecam *dev)
{
	int i;

	for (i = 0; i < PIPECAM_NURBS; i++) {
		if (!dev->urb[i])
			continue;
		kfree(dev->urb[i]->transfer_buffer);
		usb_free_urb(dev->urb[i]);
		dev->urb[i] = NULL;
	}
}

static void pipecam_kill_urbs(struct pipecam *dev)
{
	int i;

	for (i = 0; i < PIPECAM_NURBS; i++)
		if (dev->urb[i])
			usb_kill_urb(dev->urb[i]);
}

static int pipecam_alloc_urbs(struct pipecam *dev)
{
	unsigned int pipe = usb_rcvbulkpipe(dev->udev,
			PIPECAM_EP_IN & USB_ENDPOINT_NUMBER_MASK);
	int i;

	for (i = 0; i < PIPECAM_NURBS; i++) {
		void *buf;
		struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);

		if (!urb)
			goto fail;
		dev->urb[i] = urb;

		buf = kmalloc(PIPECAM_URB_SIZE, GFP_KERNEL);
		if (!buf)
			goto fail;

		usb_fill_bulk_urb(urb, dev->udev, pipe, buf,
				  PIPECAM_URB_SIZE, pipecam_urb_complete, dev);
	}
	return 0;

fail:
	pipecam_free_urbs(dev);
	return -ENOMEM;
}

/* ------------------------------------------------------------------ */
/* videobuf2                                                           */
/* ------------------------------------------------------------------ */

static int pipecam_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < PIPECAM_IMGSIZE)
			return -EINVAL;
		return 0;
	}
	*nplanes = 1;
	sizes[0] = PIPECAM_IMGSIZE;
	return 0;
}

static int pipecam_buf_prepare(struct vb2_buffer *vb)
{
	if (vb2_plane_size(vb, 0) < PIPECAM_IMGSIZE)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, PIPECAM_IMGSIZE);
	return 0;
}

static void pipecam_buf_queue(struct vb2_buffer *vb)
{
	struct pipecam *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct pipecam_buf *buf = to_pipecam_buf(to_vb2_v4l2_buffer(vb));
	unsigned long flags;

	spin_lock_irqsave(&dev->qlock, flags);
	list_add_tail(&buf->list, &dev->buf_list);
	spin_unlock_irqrestore(&dev->qlock, flags);
}

static void pipecam_return_buffers(struct pipecam *dev,
				   enum vb2_buffer_state state)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->qlock, flags);
	if (dev->cur) {
		vb2_buffer_done(&dev->cur->vb.vb2_buf, state);
		dev->cur = NULL;
		dev->cur_vaddr = NULL;
		dev->cur_off = 0;
	}
	while (!list_empty(&dev->buf_list)) {
		struct pipecam_buf *buf = list_first_entry(&dev->buf_list,
						struct pipecam_buf, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
	spin_unlock_irqrestore(&dev->qlock, flags);
}

static int pipecam_set_idle(struct pipecam *dev)
{
	int ret;

	ret = usb_set_interface(dev->udev, PIPECAM_IFNUM, PIPECAM_ALT_IDLE);
	if (ret)
		dev_warn(&dev->intf->dev,
			 "failed to restore idle altsetting: %d\n", ret);
	return ret;
}

static int pipecam_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct pipecam *dev = vb2_get_drv_priv(q);
	unsigned int pipe = usb_rcvbulkpipe(dev->udev,
			PIPECAM_EP_IN & USB_ENDPOINT_NUMBER_MASK);
	unsigned long flags;
	int i, ret, submitted = 0;

	if (!dev->present) {
		pipecam_return_buffers(dev, VB2_BUF_STATE_QUEUED);
		return -ENODEV;
	}

	spin_lock_irqsave(&dev->qlock, flags);
	dev->cur = NULL;
	dev->cur_vaddr = NULL;
	dev->cur_off = 0;
	dev->frame_bytes = 0;
	dev->fid_valid = false;
	dev->frame_synced = false;
	dev->frame_bad = false;
	dev->sequence = 0;
	dev->urb_errors = 0;
	dev->stream_state = PIPECAM_STARTING;
	spin_unlock_irqrestore(&dev->qlock, flags);

	/* Selecting the streaming altsetting is what starts the data flow. */
	ret = usb_set_interface(dev->udev, PIPECAM_IFNUM, PIPECAM_ALT_STREAM);
	if (ret < 0) {
		dev_err(&dev->intf->dev, "set_interface(alt %d) failed: %d\n",
			PIPECAM_ALT_STREAM, ret);
		goto err;
	}

	/* A previous -EPIPE leaves the endpoint halted.  Clearing it here makes
	 * the documented streamoff/streamon recovery path deterministic.
	 */
	ret = usb_clear_halt(dev->udev, pipe);
	if (ret < 0) {
		dev_err(&dev->intf->dev, "clear_halt failed: %d\n", ret);
		goto err_alt;
	}

	for (i = 0; i < PIPECAM_NURBS; i++) {
		ret = usb_submit_urb(dev->urb[i], GFP_KERNEL);
		if (ret) {
			dev_err(&dev->intf->dev,
				"URB %d submit failed: %d\n", i, ret);
			goto err_urbs;
		}
		submitted++;

		spin_lock_irqsave(&dev->qlock, flags);
		if (dev->stream_state == PIPECAM_FAILED)
			ret = -EIO;
		spin_unlock_irqrestore(&dev->qlock, flags);
		if (ret)
			goto err_urbs;
	}

	spin_lock_irqsave(&dev->qlock, flags);
	if (dev->stream_state == PIPECAM_STARTING) {
		dev->stream_state = PIPECAM_RUNNING;
		ret = 0;
	} else {
		ret = -EIO;
	}
	spin_unlock_irqrestore(&dev->qlock, flags);
	if (!ret)
		return 0;

err_urbs:
	spin_lock_irqsave(&dev->qlock, flags);
	dev->stream_state = PIPECAM_STOPPING;
	spin_unlock_irqrestore(&dev->qlock, flags);
	while (submitted--)
		usb_kill_urb(dev->urb[submitted]);
err_alt:
	pipecam_set_idle(dev);
err:
	spin_lock_irqsave(&dev->qlock, flags);
	dev->stream_state = PIPECAM_STOPPED;
	spin_unlock_irqrestore(&dev->qlock, flags);
	pipecam_return_buffers(dev, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void pipecam_stop_streaming(struct vb2_queue *q)
{
	struct pipecam *dev = vb2_get_drv_priv(q);
	unsigned long flags;

	spin_lock_irqsave(&dev->qlock, flags);
	dev->stream_state = PIPECAM_STOPPING;
	spin_unlock_irqrestore(&dev->qlock, flags);

	pipecam_kill_urbs(dev);

	/* Altsetting 0 has no IN endpoint, which stops the stream.  Skip the
	 * transfer when tearing down from disconnect (present is cleared
	 * first): the device is gone, or the core reclaims the interface
	 * right after unbind anyway.
	 */
	if (dev->present)
		pipecam_set_idle(dev);

	pipecam_return_buffers(dev, VB2_BUF_STATE_ERROR);

	spin_lock_irqsave(&dev->qlock, flags);
	dev->stream_state = PIPECAM_STOPPED;
	spin_unlock_irqrestore(&dev->qlock, flags);
}

static const struct vb2_ops pipecam_vb2_ops = {
	.queue_setup		= pipecam_queue_setup,
	.buf_prepare		= pipecam_buf_prepare,
	.buf_queue		= pipecam_buf_queue,
	.start_streaming	= pipecam_start_streaming,
	.stop_streaming		= pipecam_stop_streaming,
};

/* ------------------------------------------------------------------ */
/* V4L2 ioctls                                                         */
/* ------------------------------------------------------------------ */

static int pipecam_querycap(struct file *file, void *priv,
			    struct v4l2_capability *cap)
{
	struct pipecam *dev = video_drvdata(file);

	strscpy(cap->driver, PIPECAM_DRIVER, sizeof(cap->driver));
	strscpy(cap->card, PIPECAM_CARD, sizeof(cap->card));
	usb_make_path(dev->udev, cap->bus_info, sizeof(cap->bus_info));
	return 0;
}

static int pipecam_enum_fmt(struct file *file, void *priv,
			    struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_YUYV;
	return 0;
}

static void pipecam_fill_fmt(struct v4l2_format *f)
{
	struct v4l2_pix_format *p = &f->fmt.pix;

	p->width	= PIPECAM_WIDTH;
	p->height	= PIPECAM_HEIGHT;
	p->pixelformat	= V4L2_PIX_FMT_YUYV;
	p->field	= V4L2_FIELD_NONE;
	p->bytesperline	= PIPECAM_BPL;
	p->sizeimage	= PIPECAM_IMGSIZE;
	p->colorspace	= V4L2_COLORSPACE_SRGB;
	/* The sensor emits full-range BT.601 YCbCr (measured luma 6..254). */
	p->quantization	= V4L2_QUANTIZATION_FULL_RANGE;
	p->ycbcr_enc	= V4L2_YCBCR_ENC_601;
	p->xfer_func	= V4L2_XFER_FUNC_SRGB;
	/* Every field must be written: TRY/S_FMT replies would otherwise echo
	 * caller-supplied values (SET_CSC in particular is not supported).
	 */
	p->flags	= 0;
}

static int pipecam_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	pipecam_fill_fmt(f);
	return 0;
}

/* The device has exactly one fixed mode, so try/set simply report it. */
static int pipecam_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	pipecam_fill_fmt(f);
	return 0;
}

static int pipecam_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct pipecam *dev = video_drvdata(file);

	if (vb2_is_busy(&dev->queue))
		return -EBUSY;
	pipecam_fill_fmt(f);
	return 0;
}

static int pipecam_enum_framesizes(struct file *file, void *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_YUYV)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = PIPECAM_WIDTH;
	fsize->discrete.height = PIPECAM_HEIGHT;
	return 0;
}

static int pipecam_enum_input(struct file *file, void *priv,
			      struct v4l2_input *inp)
{
	if (inp->index)
		return -EINVAL;
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strscpy(inp->name, "Pipe Camera", sizeof(inp->name));
	return 0;
}

static int pipecam_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int pipecam_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops pipecam_ioctl_ops = {
	.vidioc_querycap		= pipecam_querycap,
	.vidioc_enum_fmt_vid_cap	= pipecam_enum_fmt,
	.vidioc_g_fmt_vid_cap		= pipecam_g_fmt,
	.vidioc_s_fmt_vid_cap		= pipecam_s_fmt,
	.vidioc_try_fmt_vid_cap		= pipecam_try_fmt,
	.vidioc_enum_framesizes		= pipecam_enum_framesizes,
	.vidioc_enum_input		= pipecam_enum_input,
	.vidioc_g_input			= pipecam_g_input,
	.vidioc_s_input			= pipecam_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations pipecam_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.unlocked_ioctl	= video_ioctl2,
};

/* ------------------------------------------------------------------ */
/* USB probe / disconnect                                              */
/* ------------------------------------------------------------------ */

static void pipecam_release(struct v4l2_device *v4l2_dev)
{
	struct pipecam *dev = container_of(v4l2_dev, struct pipecam, v4l2_dev);

	pipecam_free_urbs(dev);
	v4l2_device_unregister(&dev->v4l2_dev);
	usb_put_intf(dev->intf);
	usb_put_dev(dev->udev);
	mutex_destroy(&dev->lock);
	kfree(dev);
}

static int pipecam_validate_interface(struct usb_interface *intf)
{
	struct usb_host_interface *idle, *stream;
	struct usb_endpoint_descriptor *ep = NULL;
	int i;

	idle = usb_altnum_to_altsetting(intf, PIPECAM_ALT_IDLE);
	stream = usb_altnum_to_altsetting(intf, PIPECAM_ALT_STREAM);
	if (!idle || !stream)
		return -ENODEV;

	if (idle->desc.bInterfaceNumber != PIPECAM_IFNUM ||
	    idle->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC ||
	    idle->desc.bInterfaceSubClass != 0xf0 ||
	    idle->desc.bInterfaceProtocol != 1 ||
	    stream->desc.bInterfaceNumber != PIPECAM_IFNUM ||
	    stream->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC ||
	    stream->desc.bInterfaceSubClass != 0xf0 ||
	    stream->desc.bInterfaceProtocol != 1)
		return -ENODEV;

	/* The idle setting must really stop input, otherwise disconnect and
	 * streamoff would leave an unknown device transmitting.
	 */
	for (i = 0; i < idle->desc.bNumEndpoints; i++)
		if (usb_endpoint_dir_in(&idle->endpoint[i].desc))
			return -ENODEV;

	/* Match the protocol's exact endpoint rather than assuming that the first
	 * bulk-IN endpoint belongs to the video stream.
	 */
	for (i = 0; i < stream->desc.bNumEndpoints; i++)
		if (stream->endpoint[i].desc.bEndpointAddress == PIPECAM_EP_IN) {
			ep = &stream->endpoint[i].desc;
			break;
		}

	if (!ep || !usb_endpoint_is_bulk_in(ep) ||
	    usb_endpoint_maxp(ep) != PIPECAM_USB_PKT)
		return -ENODEV;

	return 0;
}

static int pipecam_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct pipecam *dev;
	struct vb2_queue *q;
	int ret;

	ret = pipecam_validate_interface(intf);
	if (ret) {
		dev_err(&intf->dev, "unsupported interface layout\n");
		return ret;
	}

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->udev = usb_get_dev(udev);
	/* dev->intf is dereferenced from URB completion; pin it for the
	 * lifetime of dev, which can outlast the unbind.
	 */
	dev->intf = usb_get_intf(intf);
	dev->present = true;
	mutex_init(&dev->lock);
	spin_lock_init(&dev->qlock);
	INIT_LIST_HEAD(&dev->buf_list);

	/* Make sure we start from the idle altsetting. */
	ret = usb_set_interface(udev, PIPECAM_IFNUM, PIPECAM_ALT_IDLE);
	if (ret) {
		dev_err(&intf->dev, "set_interface(alt %d) failed: %d\n",
			PIPECAM_ALT_IDLE, ret);
		goto err_free;
	}

	dev->v4l2_dev.release = pipecam_release;
	ret = v4l2_device_register(&intf->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(&intf->dev, "v4l2_device_register failed: %d\n", ret);
		goto err_free;
	}

	q = &dev->queue;
	q->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes		= VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->drv_priv		= dev;
	q->buf_struct_size	= sizeof(struct pipecam_buf);
	q->ops			= &pipecam_vb2_ops;
	q->mem_ops		= &vb2_vmalloc_memops;
	q->timestamp_flags	= V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->min_queued_buffers	= 2;
	q->lock			= &dev->lock;
	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(&intf->dev, "vb2_queue_init failed: %d\n", ret);
		goto err_v4l2;
	}

	/* Keep the transfer pool for the device lifetime.  STREAMOFF only kills
	 * submissions, so repeated capture cycles cannot fail because memory has
	 * become too fragmented to rebuild this high-order allocation pool.
	 */
	ret = pipecam_alloc_urbs(dev);
	if (ret) {
		dev_err(&intf->dev, "URB pool allocation failed\n");
		goto err_v4l2;
	}

	strscpy(dev->vdev.name, PIPECAM_CARD, sizeof(dev->vdev.name));
	dev->vdev.v4l2_dev	= &dev->v4l2_dev;
	dev->vdev.fops		= &pipecam_fops;
	dev->vdev.ioctl_ops	= &pipecam_ioctl_ops;
	dev->vdev.release	= video_device_release_empty;
	dev->vdev.queue		= q;
	dev->vdev.lock		= &dev->lock;
	dev->vdev.device_caps	= V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
				  V4L2_CAP_READWRITE;
	video_set_drvdata(&dev->vdev, dev);

	ret = video_register_device(&dev->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(&intf->dev, "video_register_device failed: %d\n", ret);
		goto err_v4l2;
	}

	usb_set_intfdata(intf, dev);
	dev_info(&intf->dev, "%s registered as /dev/video%d (%ux%u YUYV)\n",
		 PIPECAM_CARD, dev->vdev.num, PIPECAM_WIDTH, PIPECAM_HEIGHT);
	return 0;

err_v4l2:
	/* .release is armed, so this frees dev via pipecam_release(). */
	v4l2_device_put(&dev->v4l2_dev);
	return ret;
err_free:
	pipecam_free_urbs(dev);
	usb_put_intf(dev->intf);
	usb_put_dev(dev->udev);
	mutex_destroy(&dev->lock);
	kfree(dev);
	return ret;
}

/* Preserve the registered node across system sleep and USB reset, but make
 * interrupted capture explicitly recoverable instead of attempting to resume
 * a partially assembled frame.  The VB2 error is cleared by STREAMOFF; the
 * following STREAMON resets assembly state and resubmits the retained URBs.
 */
static void pipecam_quiesce(struct pipecam *dev)
{
	if (!vb2_is_streaming(&dev->queue))
		return;

	pipecam_fail_stream(dev);
	pipecam_kill_urbs(dev);
}

static int pipecam_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct pipecam *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;

	mutex_lock(&dev->lock);
	pipecam_quiesce(dev);
	mutex_unlock(&dev->lock);
	return 0;
}

static int pipecam_resume(struct usb_interface *intf)
{
	struct pipecam *dev = usb_get_intfdata(intf);
	int ret = 0;

	if (!dev)
		return 0;

	mutex_lock(&dev->lock);
	if (dev->present)
		ret = pipecam_set_idle(dev);
	mutex_unlock(&dev->lock);
	return ret;
}

static int pipecam_pre_reset(struct usb_interface *intf)
{
	struct pipecam *dev = usb_get_intfdata(intf);

	if (!dev)
		return 0;

	mutex_lock(&dev->lock);
	pipecam_quiesce(dev);
	mutex_unlock(&dev->lock);
	return 0;
}

static void pipecam_disconnect(struct usb_interface *intf)
{
	struct pipecam *dev = usb_get_intfdata(intf);

	if (!dev)
		return;

	usb_set_intfdata(intf, NULL);

	mutex_lock(&dev->lock);
	dev->present = false;
	pipecam_fail_stream(dev);
	vb2_queue_error(&dev->queue);
	mutex_unlock(&dev->lock);

	/* Stop streaming here, not at last close: URB completions must not
	 * outlive disconnect, where the USB core is about to drop its
	 * interface reference and reclaim the endpoints.
	 */
	vb2_video_unregister_device(&dev->vdev);
	v4l2_device_disconnect(&dev->v4l2_dev);

	/* Frees dev via pipecam_release() once the last user goes away; the
	 * USB references are dropped there, not here, because open fds still
	 * reach dev->queue and dev->lock after this returns.
	 */
	v4l2_device_put(&dev->v4l2_dev);
}

/*
 * Bind interface 1 only.  Interface 0 is the Apple iAP control interface and
 * is deliberately left unclaimed.
 */
static const struct usb_device_id pipecam_table[] = {
	{ USB_DEVICE_INTERFACE_NUMBER(PIPECAM_VID, PIPECAM_PID, PIPECAM_IFNUM) },
	{ }
};
MODULE_DEVICE_TABLE(usb, pipecam_table);

static struct usb_driver pipecam_driver = {
	.name		= PIPECAM_DRIVER,
	.probe		= pipecam_probe,
	.disconnect	= pipecam_disconnect,
	.suspend	= pipecam_suspend,
	.resume		= pipecam_resume,
	.reset_resume	= pipecam_resume,
	.pre_reset	= pipecam_pre_reset,
	.post_reset	= pipecam_resume,
	.id_table	= pipecam_table,
};

module_usb_driver(pipecam_driver);

MODULE_AUTHOR("Christian Reiss <email@christian-reiss.de>");
MODULE_DESCRIPTION("V4L2 driver for Look Kellyop lem01camera USB pipe camera");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
