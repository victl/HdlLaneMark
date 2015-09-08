TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    CurbScanner.cpp \
    defines.cpp \
    DynamicMap.cpp \
    HDLDisplay.cpp \
    LocalMap.cpp \
    StaticMap.cpp

include(deployment.pri)
qtcAddDeployment()

CONFIG += c++11

LIBS += `pkg-config opencv --cflags --libs` \
    -lglog

INCLUDEPATH += /usr/include/opencv \
             /usr/include/opencv2

HEADERS += \
    CurbScanner.h \
    defines.h \
    DynamicMap.h \
    Grid.h \
    HDLDisplay.h \
    LocalMap.h \
    module.h \
    StaticMap.h \
    config.h
