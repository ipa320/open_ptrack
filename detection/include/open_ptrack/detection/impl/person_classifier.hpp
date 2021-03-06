/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * person_classifier.hpp
 * Created on: Nov 30, 2012
 * Author: Matteo Munaro
 */

#include <open_ptrack/detection/person_classifier.h>

#ifndef OPEN_PTRACK_DETECTION_PERSON_CLASSIFIER_HPP_
#define OPEN_PTRACK_DETECTION_PERSON_CLASSIFIER_HPP_

template <typename PointT>
open_ptrack::detection::PersonClassifier<PointT>::PersonClassifier () {}

template <typename PointT>
open_ptrack::detection::PersonClassifier<PointT>::~PersonClassifier () {}

template <typename PointT> bool
open_ptrack::detection::PersonClassifier<PointT>::loadSVMFromFile (std::string svm_filename)
{
  std::string line;
  std::ifstream SVM_file;
  SVM_file.open(svm_filename.c_str());

  getline (SVM_file,line);      // read window_height line
  size_t tok_pos = line.find_first_of(":", 0);  // search for token ":"
  window_height_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read window_width line
  tok_pos = line.find_first_of(":", 0);  // search for token ":"
  window_width_ = std::atoi(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_offset line
  tok_pos = line.find_first_of(":", 0);  // search for token ":"
  SVM_offset_ = std::atof(line.substr(tok_pos+1, line.npos - tok_pos-1).c_str());

  getline (SVM_file,line);      // read SVM_weights line
  tok_pos = line.find_first_of("[", 0);  // search for token "["
  size_t tok_end_pos = line.find_first_of("]", 0);  // search for token "]" , end of SVM weights
  size_t prev_tok_pos;
  while (tok_pos < tok_end_pos) // while end of SVM_weights is not reached
  {
    prev_tok_pos = tok_pos;
    tok_pos = line.find_first_of(",", prev_tok_pos+1);  // search for token ","
    SVM_weights_.push_back(std::atof(line.substr(prev_tok_pos+1, tok_pos-prev_tok_pos-1).c_str()));
  }
  SVM_file.close();
  
  if (SVM_weights_.size() == 0)
  {
    PCL_ERROR ("[open_ptrack::detection::PersonClassifier::loadSVMFromFile] Invalid SVM file!\n");
    return (false);
  }
  else
  {
    return (true);
  }
}

template <typename PointT> void
open_ptrack::detection::PersonClassifier<PointT>::setSVM (int window_height, int window_width, std::vector<float> SVM_weights, float SVM_offset)
{
  window_height_ = window_height;
  window_width_ = window_width;
  SVM_weights_ = SVM_weights;
  SVM_offset_ = SVM_offset;
}

template <typename PointT> void
open_ptrack::detection::PersonClassifier<PointT>::getSVM (int& window_height, int& window_width, std::vector<float>& SVM_weights, float& SVM_offset)
{
  window_height = window_height_;
  window_width = window_width_;
  SVM_weights = SVM_weights_;
  SVM_offset = SVM_offset_;
}

template <typename PointT> void
open_ptrack::detection::PersonClassifier<PointT>::resize (PointCloudPtr& input_image,
              PointCloudPtr& output_image,
              int width,
              int height)
{
  PointT new_point;
  new_point.r = 0;
  new_point.g = 0;
  new_point.b = 0;

  // Allocate the vector of points:
  output_image->points.resize(width*height, new_point);
  output_image->height = height;
  output_image->width = width;

  // Compute scale factor:
  float scale1 = float(height) / float(input_image->height);
  float scale2 = float(width) / float(input_image->width);

  Eigen::Matrix3f T_inv;
  T_inv << 1/scale1, 0, 0,
       0, 1/scale2, 0,
       0,   0,   1;

  Eigen::Vector3f A;
  int c1, c2, f1, f2;
  PointT g1, g2, g3, g4;
  float w1, w2;
  for (unsigned int i = 0; i < height; i++)    // for every row
  {
  for (unsigned int j = 0; j < width; j++)  // for every column
  {
    A = T_inv * Eigen::Vector3f(i, j, 1);
    c1 = ceil(A(0));
    f1 = floor(A(0));
    c2 = ceil(A(1));
    f2 = floor(A(1));

    if ( (f1 < 0) ||
       (c1 < 0) ||
       (f1 >= input_image->height) ||
       (c1 >= input_image->height) ||
       (f2 < 0) ||
       (c2 < 0) ||
       (f2 >= input_image->width) ||
       (c2 >= input_image->width))
    { // if out of range, continue
    continue;
    }

    g1 = (*input_image)(f2, c1);
    g3 = (*input_image)(f2, f1);
    g4 = (*input_image)(c2, f1);
    g2 = (*input_image)(c2, c1);

    w1 = (A(0) - f1);
    w2 = (A(1) - f2);
    new_point.r = int((1 - w1) * ((1 - w2) * g1.r + w2 * g4.r) + w1 * ((1 - w2) * g3.r + w2 * g4.r));
    new_point.g = int((1 - w1) * ((1 - w2) * g1.g + w2 * g4.g) + w1 * ((1 - w2) * g3.g + w2 * g4.g));
    new_point.b = int((1 - w1) * ((1 - w2) * g1.b + w2 * g4.b) + w1 * ((1 - w2) * g3.b + w2 * g4.b));

    // Insert the point in the output image:
    (*output_image)(j,i) = new_point;
  }
  }
}

template <typename PointT> double
open_ptrack::detection::PersonClassifier<PointT>::evaluate (PointCloudPtr& image, Eigen::Vector3f& bottom, Eigen::Vector3f& top,
              Eigen::Vector3f& centroid)
{
    float height_person = bottom(1) - top(1);
    float xc = centroid(0);
    float yc = centroid(1);

    if (SVM_weights_.size() == 0)
     {
         PCL_ERROR ("[pcl::people::PersonClassifier::evaluate] SVM has not been set!\n");
         return (-1000);
     }

     int height = floor((height_person * window_height_) / (0.75 * window_height_) + 0.5);  // floor(i+0.5) = round(i)
     int width = floor((height_person * window_width_) / (0.75 * window_height_) + 0.5);
     int xmin = floor(xc - width / 2 + 0.5);
     int ymin = floor(yc - height / 2 + 0.5);
     double confidence;

     if (height > 0)
     {
      // Make the image match the correct size (used in the training stage):
       PointCloudPtr sample(new pcl::PointCloud<pcl::RGB>);
       resize(image, sample, window_width_, window_height_);

       // Convert the image to array of float:
       float* sample_float = new float[sample->width * sample->height * 3];
       int delta = sample->height * sample->width;
       for(int row = 0; row < sample->height; row++)
       {
         for(int col = 0; col < sample->width; col++)
         {
           sample_float[row + sample->height * col] = ((float) ((*sample)(col, row).r))/255; //ptr[col * 3 + 2];
           sample_float[row + sample->height * col + delta] = ((float) ((*sample)(col, row).g))/255; //ptr[col * 3 + 1];
           sample_float[row + sample->height * col + delta * 2] = (float) (((*sample)(col, row).b))/255; //ptr[col * 3];
         }
       }

       // Calculate HOG descriptor:
       pcl::people::HOG hog;
       float *descriptor = (float*) calloc(SVM_weights_.size(), sizeof(float));
       hog.compute(sample_float, descriptor);

       // Calculate confidence value by dot product:
       confidence = 0.0;
       for(unsigned int i = 0; i < SVM_weights_.size(); i++)
       {
         confidence += SVM_weights_[i] * descriptor[i];
       }
       // Confidence correction:
       confidence -= SVM_offset_;

       delete[] descriptor;
       delete[] sample_float;
     }
     else
     {
       confidence = std::numeric_limits<double>::quiet_NaN();
     }

     return confidence;
}


#endif /* OPEN_PTRACK_DETECTION_PERSON_CLASSIFIER_HPP_ */
