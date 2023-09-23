#include "utils.h"
#include <boost/filesystem.hpp>
#include <vector>

namespace fs = boost::filesystem;
namespace utils
{

    void ReadKittiBinCloudByPath(const std::string & path, Cloud & cloud)
    {
        std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
        if (file.good())
        {
            file.seekg(0, std::ios::beg);
            float intensity = 0;
            for (int i = 0; file.good() && !file.eof(); ++i)
            {
                point pt;
                file.read(reinterpret_cast<char*>(&pt.x()), sizeof(float));
                file.read(reinterpret_cast<char*>(&pt.y()), sizeof(float));
                file.read(reinterpret_cast<char*>(&pt.z()), sizeof(float));
                file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
                cloud.emplace_back(pt);
            }
            file.close();
        }
    }
    Cloud::Ptr ReadKittiBinCloudByPath(const std::string & path)
    {
        Cloud::Ptr cloud(new Cloud);
        std::fstream file(path.c_str(), std::ios::in | std::ios::binary);
        if (file.good())
        {
            file.seekg(0, std::ios::beg);
            float intensity = 0;
            for (int i = 0; file.good() && !file.eof(); ++i)
            {
                point pt;
                file.read(reinterpret_cast<char*>(&pt.x()), sizeof(float));
                file.read(reinterpret_cast<char*>(&pt.y()), sizeof(float));
                file.read(reinterpret_cast<char*>(&pt.z()), sizeof(float));
                file.read(reinterpret_cast<char*>(&intensity), sizeof(float));
                // if (std::atan2(pt.y(), pt.x()) < M_PI / 4 && std::atan2(pt.y(), pt.x()) > -M_PI / 4)  // delete some points
                // if (pt.x() > 0 && pt.x() < 70.2f && pt.y() > -40.0f && pt.y() < 40.0f && pt.z() > -3.0f && pt.z() < 1.0f)
                cloud->emplace_back(pt);
            }
            file.close();
        }
        return cloud;
    }

    std::vector<Cloud::Ptr> ReadKittiLabelBoxByPath(const std::string & path,
                                                    Eigen::MatrixXd MATRIX_T_VELO_2_CAM,
                                                    Eigen::MatrixXd MATRIX_R_RECT_0,
                                                    int dataLen) {
        fprintf(stderr, "label2 path: %s\n", path.c_str());
        FILE *fp = fopen(path.c_str(), "r");
        if (fp == nullptr)
        {
            fprintf(stderr, "open tracking label file error!\n");
            exit(0);
        }
        int frame = 0, track_id, truncated, occluded;

        float alpha, bboxL, bboxR, bboxU, bboxB, h, w, l, x, y, z, rotation_y, score;
        float typeValue;
        std::vector<Cloud::Ptr> resultBoxs;
        if (dataLen == 15) {
            while(!feof(fp)) {
                char type[255];
                int numRead = fscanf(fp, "%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                        type, &truncated, &occluded, &alpha, &bboxL, &bboxR, &bboxU, &bboxB,
                                         &h, &w, &l, &x, &y, &z, &rotation_y);/*, &score);*/
                if (numRead != 15) {
                    fprintf(stderr, "numRead %d\n", numRead);
                }
                point lidarCoord(x, y, z);
                lidarCoord = camera_to_lidar(x, y, z, MATRIX_T_VELO_2_CAM, MATRIX_R_RECT_0);
                Cloud bbox = center_to_corner_box3d(lidarCoord, h, w, l, rotation_y);
                std::vector<Cloud::Ptr> bboxs;
                Cloud::Ptr bboxPtr(new Cloud);
                *bboxPtr = bbox;
                resultBoxs.emplace_back(bboxPtr);
            }
        } else if (dataLen == 16)
        {
            while(!feof(fp)) {
                char type[255];
                int numRead = fscanf(fp, "%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                                     type, &truncated, &occluded, &alpha, &bboxL, &bboxR, &bboxU, &bboxB,
                                     &h, &w, &l, &x, &y, &z, &rotation_y, &score);
                if (numRead != 15) {
                    fprintf(stderr, "numRead %d\n", numRead);
                }
                if (score > 0.7F) {// draw car only ???
                    point lidarCoord(x, y, z);
                    lidarCoord = camera_to_lidar(x, y, z, MATRIX_T_VELO_2_CAM, MATRIX_R_RECT_0);
                    Cloud bbox = center_to_corner_box3d(lidarCoord, h, w, l, rotation_y);
                    std::vector<Cloud::Ptr> bboxs;
                    Cloud::Ptr bboxPtr(new Cloud);
                    *bboxPtr = bbox;
                    resultBoxs.emplace_back(bboxPtr);
                }
            }
        } else {
            while (!feof(fp))
            {
                char type[255];
                // int numRead = fscanf(fp, "%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
                //        type, &truncated, &occluded, &alpha, &bboxL, &bboxR, &bboxU, &bboxB,
                //                         &h, &w, &l, &x, &y, &z, &rotation_y);/*, &score);*/
                int numRead = fscanf(fp, "%f %f %f %f %f %f %f %f %f\n",
                                          &x, &y, &z, &l, &w, &h, &rotation_y, &score, &typeValue);
                z = z - h / 2; // pvrcnn to kitti stand
                rotation_y = -(rotation_y + M_PI / 2); // pvrcnn to kitti stand
                // if (numRead != 15)
                //     fprintf(stderr, "numRead %d\n", numRead);
                // if (strcmp(type, "Car") == 0 || strcmp(type, "Van") == 0)
                if (numRead != 9)
                    fprintf(stderr, "numRead %d\n", numRead);
//                if (abs(typeValue - 1.0) < 0.001F && score > 0.5F) // draw car only ???
                if (score > 0.7F) // draw car only ???
                {
                    point lidarCoord(x, y, z);
                    // lidarCoord = camera_to_lidar(x, y, z, MATRIX_T_VELO_2_CAM, MATRIX_R_RECT_0);
                    Cloud bbox = center_to_corner_box3d(lidarCoord, h, w, l, rotation_y);
                    std::vector<Cloud::Ptr> bboxs;
                    Cloud::Ptr bboxPtr(new Cloud);
                    *bboxPtr = bbox;
                    resultBoxs.emplace_back(bboxPtr);
                }
            }
        }
        fclose(fp);
        return resultBoxs;
    }

    void ReadKittiImageByPath(const std::string & path, cv::Mat & img)
    {
        // fprintf(stderr, "path: %s\n", path.c_str());
        img = cv::imread(path);
        if (img.empty())
        {
            printf("no kitti image was readed!\n");
            return;
        }
    }

    void ReadKittiFileByDir(const std::string & dir, std::vector<std::string> &fileNames)
    {
        // printf("kitti velodyne path :%s\n", dir.c_str());
        fs::path velo_path(dir);
        fs::directory_iterator end;
        for (fs::directory_iterator fileIt(velo_path); fileIt != end; ++fileIt)
        {
            std::string filename =  fileIt->path().string();
            fileNames.emplace_back(filename);         
        }
    }

    QImage MatToQImage(const cv::Mat &image) 
    {
        auto qimage = QImage(image.cols, image.rows, QImage::Format_RGB888);
        if (image.type() == CV_32F) 
        {
            for (int r = 0; r < image.rows; ++r) 
            {
                for (int c = 0; c < image.cols; ++c) 
                {
                    if (image.at<float>(r, c) == 666) 
                    {
                        auto color = qRgb(0, 200, 0);
                        qimage.setPixel(c, r, color);
                        continue;
                    }
                    const float &val = image.at<float>(r, c) * 10;
                    auto color = qRgb(val, val, val);
                    qimage.setPixel(c, r, color);
                }
            }
        } 
        else 
        {
            for (int r = 0; r < image.rows; ++r) 
            {
                for (int c = 0; c < image.cols; ++c) 
                {
                    auto val = image.at<cv::Vec3b>(r, c);
                    auto color = qRgb(val[0], val[1], val[2]);
                    qimage.setPixel(c, r, color);
                }
            }
        }
        return qimage;
    }

    point camera_to_lidar(float x, float y, float z, 
				const Eigen::MatrixXd & T_VELO_2_CAM, 
				const Eigen::MatrixXd & R_RECT_0)
    {
        Eigen::VectorXd p = Eigen::VectorXd(4, 1);
        p << x, y, z, 1;
        p = R_RECT_0.inverse() * p;
        p = T_VELO_2_CAM.inverse() * p;
        return point(p(0), p(1), p(2));
    }

    Cloud center_to_corner_box3d(const point & center, 
							 const float & h,
							 const float & w,
							 const float & l,
							 float yaw)
    {
        point pt1(-l / 2,  w / 2, 0.0f);
        point pt2(-l / 2, -w / 2, 0.0f);
        point pt3( l / 2, -w / 2, 0.0f);
        point pt4( l / 2,  w / 2, 0.0f);

        point pt5(-l / 2,  w / 2, h);
        point pt6(-l / 2, -w / 2, h);
        point pt7( l / 2, -w / 2, h);
        point pt8( l / 2,  w / 2, h);

        Cloud bbox;
        bbox.emplace_back(pt1);
        bbox.emplace_back(pt2);
        bbox.emplace_back(pt3);
        bbox.emplace_back(pt4);
        bbox.emplace_back(pt5);
        bbox.emplace_back(pt6);
        bbox.emplace_back(pt7);
        bbox.emplace_back(pt8);
        
        assert(bbox.size() == 8);

        yaw = -yaw - M_PI / 2;
        while (yaw >= M_PI / 2)
            yaw -= M_PI;
        while (yaw < -M_PI / 2)
            yaw += M_PI;

        point tmp;
        for (int idx = 0; idx < bbox.size(); ++idx)
        {
            tmp.x() =  bbox[idx].x() * cos(yaw) - bbox[idx].y() * sin(yaw) + center.x();
            tmp.y() =  bbox[idx].x() * sin(yaw) + bbox[idx].y() * cos(yaw) + center.y();
            tmp.z() =  bbox[idx].z() + center.z();
            bbox[idx] = tmp;
        }

        return bbox;
    }
}