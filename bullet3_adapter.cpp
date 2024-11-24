#include "vbropencv_adapter.h"

VBR::VideoAdapter::VideoAdapter()
{
}

VBR::VideoAdapter::~VideoAdapter()
{
    close();
}

bool VBR::VideoAdapter::open(const std::string &filename)
{
    // ensure that no other video capture object is opened
    close();

    // create a new video capture object
    m_video_capture = std::unique_ptr<cv::VideoCapture>(new cv::VideoCapture(filename));
    return is_opened();
}

void VBR::VideoAdapter::close()
{
    if (is_opened()) {
        // release the video capture object and reset the pointer
        m_video_capture->release();
        m_video_capture.reset();
    }
}

int VBR::VideoAdapter::get_width() const
{
    if (is_opened()) {
        return m_video_capture->get(cv::CAP_PROP_FRAME_WIDTH);
    }
    return 0;
}

int VBR::VideoAdapter::get_height() const
{
    if (is_opened()) {
        return m_video_capture->get(cv::CAP_PROP_FRAME_HEIGHT);
    }
    return 0;
}

int VBR::VideoAdapter::get_fps() const
{
    if (is_opened()) {
        return m_video_capture->get(cv::CAP_PROP_FPS);
    }
    return 0;
}

bool VBR::VideoAdapter::read()
{
    if (is_opened()) {
        if (m_video_capture->read(m_frame)) {
            // Convert from BGR to RGB
            cv::cvtColor(m_frame, m_frame, cv::COLOR_BGR2RGB);
            return true;
        }
    }
    return false;
}