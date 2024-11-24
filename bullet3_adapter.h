#ifndef VBROPENCV_ADAPTER_H
#define VBROPENCV_ADAPTER_H

#include <opencv2/opencv.hpp>
#include <memory>

namespace VBR
{
    class VideoAdapter
    {
    public:
        VideoAdapter();
        ~VideoAdapter();

        bool open(const std::string& filename);
        void close();

        bool is_opened() const {
            return m_video_capture != nullptr && m_video_capture->isOpened();
        }

        int get_width() const;
        int get_height() const;
        int get_fps() const;

        // read the next frame from the video capture object
        bool read();

        // returns the current frame
        const cv::Mat& get_frame() const {
            return m_frame;
        }

        // current frame raw data
        const unsigned char* get_data() const {
            return m_frame.data;
        }

        // current frame size
        int get_size() const {
            return m_frame.total() * m_frame.elemSize();
        }


    private:
        std::unique_ptr<cv::VideoCapture> m_video_capture;
        cv::Mat m_frame;
    };
}

#endif // VBROPENCV_ADAPTER_H
