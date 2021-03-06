#-------------------------------------------------
#
# Project created by QtCreator 2016-12-02T08:26:33
#
#-------------------------------------------------

QT       += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = modeling
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


CONFIG(debug, debug | release){
    DST = debug
}else{
    DST = release
}

SOURCES += main.cpp\
        mainwindow.cpp \
    glview.cpp \
    model.cpp \
    vobjcontainer.cpp \
    simple_xml.cpp \
    customslider.cpp \
    joystickemulate.cpp \
    modelroute.cpp

HEADERS  += mainwindow.h \
    glview.h \
    model.h \
    custom_types.h \
    vobjcontainer.h \
    simple_xml.hpp \
    customslider.h \
    joystickemulate.h \
    modelroute.h \
    pid_control.h

FORMS    += mainwindow.ui \
    glview.ui

win32{
    LIBS += -lopengl32 -lglu32
}else{
    LIBS += -lGL -lGLU
}

RESOURCES += \
    modeling.qrc

MOC_DIR = tmp/$$DST/moc
OBJECTS_DIR = tmp/$$DST/obj
UI_DIR = tmp/$$DST/ui
RCC_DIR = tmp/$$DST/rcc
