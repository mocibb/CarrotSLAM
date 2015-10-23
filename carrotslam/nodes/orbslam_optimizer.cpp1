/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   orbslam_optimizer.cpp
 * Date:   2015.10.04
 * Func:   ORB-SLAM tracking
 *
 *
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
 */

#include "nodes/orbslam_optimizer.h"
#include "types/feature.h"
#include "core/common.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <glog/logging.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

using namespace std;
using namespace Eigen;
using namespace carrotslam;

namespace g2o {
Vector2d project2d(const Vector3d& v)  {
  Vector2d res;
  res(0) = v(0)/v(2);
  res(1) = v(1)/v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v)  {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

EdgeSE3ProjectXYZ::EdgeSE3ProjectXYZ() : BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>() {
}

bool EdgeSE3ProjectXYZ::read(std::istream& is){
  for (int i=0; i<2; i++){
    is >> _measurement[i];
  }
  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool EdgeSE3ProjectXYZ::write(std::ostream& os) const {

  for (int i=0; i<2; i++){
    os << measurement()[i] << " ";
  }

  for (int i=0; i<2; i++)
    for (int j=i; j<2; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}


void EdgeSE3ProjectXYZ::linearizeOplus() {
  VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
  SE3Quat T(vj->estimate());
  VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
  Vector3d xyz = vi->estimate();
  Vector3d xyz_trans = T.map(xyz);

  double x = xyz_trans[0];
  double y = xyz_trans[1];
  double z = xyz_trans[2];
  double z_2 = z*z;

  Matrix<double,2,3> tmp;
  tmp(0,0) = fx;
  tmp(0,1) = 0;
  tmp(0,2) = -x/z*fx;

  tmp(1,0) = 0;
  tmp(1,1) = fy;
  tmp(1,2) = -y/z*fy;

  _jacobianOplusXi =  -1./z * tmp * T.rotation().toRotationMatrix();

  _jacobianOplusXj(0,0) =  x*y/z_2 *fx;
  _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *fx;
  _jacobianOplusXj(0,2) = y/z *fx;
  _jacobianOplusXj(0,3) = -1./z *fx;
  _jacobianOplusXj(0,4) = 0;
  _jacobianOplusXj(0,5) = x/z_2 *fx;

  _jacobianOplusXj(1,0) = (1+y*y/z_2) *fy;
  _jacobianOplusXj(1,1) = -x*y/z_2 *fy;
  _jacobianOplusXj(1,2) = -x/z *fy;
  _jacobianOplusXj(1,3) = 0;
  _jacobianOplusXj(1,4) = -1./z *fy;
  _jacobianOplusXj(1,5) = y/z_2 *fy;
}

Vector2d EdgeSE3ProjectXYZ::cam_project(const Vector3d & trans_xyz) const{
  Vector2d proj = project2d(trans_xyz);
  Vector2d res;
  res[0] = proj[0]*fx + cx;
  res[1] = proj[1]*fy + cy;
  return res;
}
} // namespace g2o

namespace carrotslam {
namespace orbslam{
  //HELPER METHOD
  g2o::SE3Quat static toSE3Quat(const Sophus::SE3f &se3) {
    return g2o::SE3Quat(se3.rotationMatrix(), se3.translation());
  }
  Sophus::SE3f static fromSE3Quat(const g2o::SE3Quat &se3q) {
    return Sophus::SE3d(se3q.rotation().toRotationMatrix(), se3q.translation()).cast<float>();
  }

  int PoseOptimization(Frame *pFrame)
  {
      g2o::SparseOptimizer optimizer;
      g2o::BlockSolverX::LinearSolverType * linearSolver;

      linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

      g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

      g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
      optimizer.setAlgorithm(solver);

      optimizer.setVerbose(false);

      int nInitialCorrespondences=0;

      // SET FRAME VERTEX
      g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
      vSE3->setEstimate(toSE3Quat(pFrame->T_f_w));
      vSE3->setId(0);
      vSE3->setFixed(false);
      optimizer.addVertex(vSE3);

      // SET MAP POINT VERTICES
      vector<g2o::EdgeSE3ProjectXYZ*> vpEdges;
      vector<g2o::VertexSBAPointXYZ*> vVertices;
      vector<float> vInvSigmas2;
      vector<size_t> vnIndexEdge;

      //const int N = pFrame->mvpMapPoints.size();
      const int N = pFrame->features.size();
      vpEdges.reserve(N);
      vVertices.reserve(N);
      vInvSigmas2.reserve(N);
      vnIndexEdge.reserve(N);

      const float delta = sqrt(5.991);

      for(int i=0; i<N; i++)
      {
          //MapPoint* pMP = pFrame->mvpMapPoints[i];
          FeaturePtr pMP = pFrame->features[i];
          if(pMP)
          {
              g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
              //vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
              vPoint->setEstimate(pMP->pos);
              vPoint->setId(i+1);
              vPoint->setFixed(true);
              optimizer.addVertex(vPoint);
              vVertices.push_back(vPoint);

              nInitialCorrespondences++;
              //pFrame->mvbOutlier[i] = false;
              pMP->is_outlier = false;

              //SET EDGE
              //Eigen::Matrix<double,2,1> obs;
              //cv::KeyPoint kpUn = pFrame->mvKeysUn[i];
              //obs << kpUn.pt.x, kpUn.pt.y;

              g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

              e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(i+1)));
              e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
              //e->setMeasurement(obs);
              e->setMeasurement(pMP->px);
              //const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
              const float invSigma2 = Feature::inv_level_sigma2[pMP->level];
              e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

              g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
              e->setRobustKernel(rk);
              rk->setDelta(delta);

              e->fx = pFrame->cam->fx;
              e->fy = pFrame->cam->fy;
              e->cx = pFrame->cam->cx;
              e->cy = pFrame->cam->cy;

              optimizer.addEdge(e);

              vpEdges.push_back(e);
              vInvSigmas2.push_back(invSigma2);
              vnIndexEdge.push_back(i);
          }

      }

      // We perform 4 optimizations, decreasing the inlier region
      // From second to final optimization we include only inliers in the optimization
      // At the end of each optimization we check which points are inliers
      const float chi2[4]={9.210,7.378,5.991,5.991};
      const int its[4]={10,10,7,5};

      int nBad=0;
      for(size_t it=0; it<4; it++)
      {
          optimizer.initializeOptimization();
          optimizer.optimize(its[it]);
          DLOG(INFO) << "iter: " << it << "\t chi2: " << optimizer.chi2() << endl;

          nBad=0;
          for(size_t i=0, iend=vpEdges.size(); i<iend; i++)
          {
              g2o::EdgeSE3ProjectXYZ* e = vpEdges[i];

              const size_t idx = vnIndexEdge[i];

              //if(pFrame->mvbOutlier[idx])
              if(pFrame->features[idx]->is_outlier)
              {
                  e->setInformation(Eigen::Matrix2d::Identity()*vInvSigmas2[i]);
                  e->computeError();
              }

              if(e->chi2()>chi2[it])
              {
                  //pFrame->mvbOutlier[idx]=true;
                  pFrame->features[idx]->is_outlier = true;
                  e->setInformation(Eigen::Matrix2d::Identity()*1e-10);
                  nBad++;
              }
              else if(e->chi2()<=chi2[it])
              {
                  //pFrame->mvbOutlier[idx]=false;
                  pFrame->features[idx]->is_outlier=false;
              }
          }

          if(optimizer.edges().size()<10)
              break;
      }

      DLOG(INFO) << "outlier: " << nBad << "/" << nInitialCorrespondences << endl;

      // Recover optimized pose and return number of inliers
      g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
      g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
      //cv::Mat pose = Converter::toCvMat(SE3quat_recov);
      //pose.copyTo(pFrame->mTcw);
      pFrame->T_f_w = fromSE3Quat(SE3quat_recov);

      return nInitialCorrespondences-nBad;
  }
}
} // namespace carrotslam
