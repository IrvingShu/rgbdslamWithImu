LIBS += -L/home/young/opencv2.9/lib -lopencv_calib3d -lopencv_core -lopencv_flann -lopencv_imgproc -lopencv_features2d -lopencv_highgui -lopencv_nonfree\
        -L/usr/lib/x86_64-linux-gnu -lboost_system \
        -L/usr/lib -lpcl_io -lpcl_visualization -lpcl_common -lpcl_filters

INCLUDEPATH += /home/young/opencv2.9/include \
               /usr/include/vtk-5.8 \
               /usr/include/pcl-1.7 \
               /usr/include/eigen3

SOURCES += \
    slambase.cpp \
    visualodometry.cpp

HEADERS += \
    slambase.h
