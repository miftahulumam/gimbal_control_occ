from ImageConvert import IMGCNV_SOpenParam, IMGCNV_ConvertToBGR24
from MVSDK import GENICAM_StreamSourceInfo, pointer, GENICAM_StreamSource, GENICAM_createStreamSource, byref, GENICAM_AcquisitionControlInfo, \
    GENICAM_EnumNode, GENICAM_EnumNodeInfo, GENICAM_createEnumNode, c_ulonglong, c_int, GENICAM_EGrabStrategy, GENICAM_Frame, c_uint, \
        c_buffer, memmove, c_char_p, EPixelType, cast, c_void_p
import struct
import time
import datetime
import numpy
import cv2
import gc

from ct_cam_helper import enumCameras, openCamera, closeCamera, setExposureTime, setFrameRate

def grab_image(ori_frame_queue):    
    # 发现相机
    cameraCnt, cameraList = enumCameras()
    if cameraCnt is None:
        return -1
    
    # 显示相机信息
    for index in range(0, cameraCnt):
        camera = cameraList[index]
        print("\nCamera Id = " + str(index))
        print("Key           = " + str(camera.getKey(camera)))
        print("vendor name   = " + str(camera.getVendorName(camera)))
        print("Model  name   = " + str(camera.getModelName(camera)))
        print("Serial number = " + str(camera.getSerialNumber(camera)))
        
    camera = cameraList[0]

    # 打开相机
    nRet = openCamera(camera)
    if ( nRet != 0 ):
        print("openCamera fail.")
        return -1
        
    # 创建流对象
    streamSourceInfo = GENICAM_StreamSourceInfo()
    streamSourceInfo.channelId = 0
    streamSourceInfo.pCamera = pointer(camera)
      
    streamSource = pointer(GENICAM_StreamSource())
    nRet = GENICAM_createStreamSource(pointer(streamSourceInfo), byref(streamSource))
    if ( nRet != 0 ):
        print("create StreamSource fail!")
        return -1
    
    # setExposureTime(camera, 1500)
    # setFrameRate(camera, 250.0)
    
    # 通用属性设置:设置触发模式为off --根据属性类型，直接构造属性节点。如触发模式是 enumNode，构造enumNode节点
    # 自由拉流：TriggerMode 需为 off
    trigModeEnumNode = pointer(GENICAM_EnumNode())
    trigModeEnumNodeInfo = GENICAM_EnumNodeInfo() 
    trigModeEnumNodeInfo.pCamera = pointer(camera)
    trigModeEnumNodeInfo.attrName = b"TriggerMode"
    nRet = GENICAM_createEnumNode(byref(trigModeEnumNodeInfo), byref(trigModeEnumNode))
    if ( nRet != 0 ):
        print("create TriggerMode Node fail!")
        # 释放相关资源
        streamSource.contents.release(streamSource) 
        return -1
    
    nRet = trigModeEnumNode.contents.setValueBySymbol(trigModeEnumNode, b"Off")
    if ( nRet != 0 ):
        print("set TriggerMode value [Off] fail!")
        # 释放相关资源
        trigModeEnumNode.contents.release(trigModeEnumNode)
        streamSource.contents.release(streamSource) 
        return -1
      
    # 需要释放Node资源    
    trigModeEnumNode.contents.release(trigModeEnumNode) 

    # 开始拉流
    nRet = streamSource.contents.startGrabbing(streamSource, c_ulonglong(0), \
                                               c_int(GENICAM_EGrabStrategy.grabStrartegySequential))
    if( nRet != 0):
        print("startGrabbing fail!")
        # 释放相关资源
        streamSource.contents.release(streamSource)   
        return -1

    isGrab = True

    while isGrab :
        # 主动取图
        frame = pointer(GENICAM_Frame())
        nRet = streamSource.contents.getFrame(streamSource, byref(frame), c_uint(1000))
        if ( nRet != 0 ):
            print("getFrame fail! Timeout:[1000]ms")
            # 释放相关资源
            streamSource.contents.release(streamSource)   
            return -1 
        else:
            # print("getFrame success BlockId = [" + str(frame.contents.getBlockId(frame)) + "], get frame time: " + str(datetime.datetime.now()))
            pass
          
        nRet = frame.contents.valid(frame)
        if ( nRet != 0 ):
            print("frame is invalid!")
            # 释放驱动图像缓存资源
            frame.contents.release(frame)
            # 释放相关资源
            streamSource.contents.release(streamSource)
            return -1 

        # 给转码所需的参数赋值
        imageParams = IMGCNV_SOpenParam()
        imageParams.dataSize    = frame.contents.getImageSize(frame)
        imageParams.height      = frame.contents.getImageHeight(frame)
        imageParams.width       = frame.contents.getImageWidth(frame)
        imageParams.paddingX    = frame.contents.getImagePaddingX(frame)
        imageParams.paddingY    = frame.contents.getImagePaddingY(frame)
        imageParams.pixelForamt = frame.contents.getImagePixelFormat(frame)

        # 将裸数据图像拷出
        imageBuff = frame.contents.getImage(frame)
        userBuff = c_buffer(b'\0', imageParams.dataSize)
        memmove(userBuff, c_char_p(imageBuff), imageParams.dataSize)

        # 释放驱动图像缓存
        frame.contents.release(frame)

        # 如果图像格式是 Mono8 直接使用
        if imageParams.pixelForamt == EPixelType.gvspPixelMono8:
            grayByteArray = bytearray(userBuff)
            cvImage = numpy.array(grayByteArray).reshape(imageParams.height, imageParams.width)
        else:
            # 转码 => BGR24
            rgbSize = c_int()
            rgbBuff = c_buffer(b'\0', imageParams.height * imageParams.width * 3)

            nRet = IMGCNV_ConvertToBGR24(cast(userBuff, c_void_p), \
                                         byref(imageParams), \
                                         cast(rgbBuff, c_void_p), \
                                         byref(rgbSize))
            # nRet = IMGCNV_ConvertToMono8(cast(userBuff, c_void_p), \
            #                              byref(imageParams), \
            #                              cast(rgbBuff, c_void_p), \
            #                              byref(rgbSize))
    
            colorByteArray = bytearray(rgbBuff)
            cvImage = numpy.array(colorByteArray).reshape(imageParams.height, imageParams.width, 3)
            # cvImage = numpy.array(colorByteArray).reshape(imageParams.height, imageParams.width, 1)
       # --- end if ---

        ori_frame_queue.put(cvImage)
        # cv2.imshow('myWindow', cvImage)
        # gc.collect()

        # if (cv2.waitKey(1) >= 0):
        #     isGrab = False
        #     break
    # --- end while ---

    # cv2.destroyAllWindows()

    # 停止拉流
    nRet = streamSource.contents.stopGrabbing(streamSource)
    if ( nRet != 0 ):
        print("stopGrabbing fail!")
        # 释放相关资源
        streamSource.contents.release(streamSource)  
        return -1

    # 关闭相机
    nRet = closeCamera(camera)
    if ( nRet != 0 ):
        print("closeCamera fail")
        # 释放相关资源
        streamSource.contents.release(streamSource)   
        return -1
     
    # 释放相关资源
    streamSource.contents.release(streamSource)    
    
    return 0
    
# if __name__=="__main__": 

#     nRet = demo()
#     if nRet != 0:
#         print("Some Error happend")
#     print("--------- Demo end ---------")
#     # 3s exit
#     time.sleep(0.5)
