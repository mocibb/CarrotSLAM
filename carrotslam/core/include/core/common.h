/*!
 * Author: mocibb mocibb@163.com
 * Group:  CarrotSLAM https://github.com/mocibb/CarrotSLAM
 * Name:   common.h
 * Date:   2015.09.30
 * Func:   common function used in CarrotSLAM
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
#ifndef CORE_COMMON_H_
#define CORE_COMMON_H_

#include <iostream>
#include <string>
#include <glog/logging.h>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace carrotslam {

/** \brief Class to measure the time spent in a scope
 *
 * To use this class, e.g. to measure the time spent in a function,
 * just create an instance at the beginning of the function. Example:
 *
 * \code
 * {
 *   ScopeTime t1 (LOG(INFO), "calculation");
 *
 *   // ... perform calculation here
 * }
 * \endcode
 *
 * \ingroup common
 */
class ScopeTime {
 public:
  inline ScopeTime(std::ostream& out, const std::string& title)
      : out_(out),
        title_(title) {
    start_time_ = boost::posix_time::microsec_clock::local_time();
  }

  inline ScopeTime(std::ostream& out)
      : ScopeTime(out, "") {
  }
  inline ScopeTime(const std::string& title)
      : ScopeTime(std::cerr, title) {
  }
  inline ScopeTime()
      : ScopeTime(std::cerr, "") {
  }


  inline ~ScopeTime() {
    double val = this->getTime();
    out_ << title_ << " took " << val << "ms.\n";
  }

  /** \brief Retrieve the time in milliseconds spent since the last call to \a reset(). */
  inline double getTime() {
    boost::posix_time::ptime end_time =
        boost::posix_time::microsec_clock::local_time();
    return (static_cast<double>(((end_time - start_time_).total_milliseconds())));
  }

 protected:
  boost::posix_time::ptime start_time_;
  std::string title_;
  std::ostream & out_;
};
}

#endif /* CORE_COMMON_H_ */
