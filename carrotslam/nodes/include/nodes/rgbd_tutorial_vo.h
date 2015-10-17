/*************************************************************************
	> File Name: include/nodes/rgbd_tutorial_vo.h
	> Author: gaoxiang
	> Mail: gaoxiang12@mails.tsinghua.edu.cn
	> Created Time: 2015年10月11日 星期日 16时53分36秒

 * The MIT License (MIT)
 * Copyright (c) 2015 CarrotSLAM Group
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ************************************************************************/

#ifndef _INCLUDE/NODES/RGBD_TUTORIAL_VO_H
#define _INCLUDE/NODES/RGBD_TUTORIAL_VO_H

#include <core/carrot_slam.h>

// std c++
#include <iostream>
#include <fstream>
#include <vector>

// opencv 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

// pcl
// 然而暂时用不到pcl

// namespace carrotslam {

/** \class RGBDTutorial_VO
 * \brief 一起做系列的VO类
 */

class RGBDTutorial_VO : public ISLAMNode
{
    public:
    /**
     * constructor
     */
}
// }
#endif
