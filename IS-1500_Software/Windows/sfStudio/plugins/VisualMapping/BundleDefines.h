/*
 * BundleDefines.h
 *
 *  Created on: May 28, 2011
 *      Author: hongsheng_zhang
 */

#ifndef BUNDLEDEFINES_H_
#define BUNDLEDEFINES_H_

//BUILD_SHARED_LIBS
#if defined (_WIN32)
  #if defined(Bundle_EXPORTS)
    #define  BUNDLE_API __declspec(dllexport)
  #else
    #define  BUNDLE_API __declspec(dllimport)
  #endif /* testDLL_EXPORTS */
#else /* defined (_WIN32) */
 #define BUNDLE_API
#endif

#endif /* BUNDLEDEFINES_H_ */
