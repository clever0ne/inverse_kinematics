TEMPLATE = app
CONFIG += console c++17
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/main/kinematics_solver.cpp \
    src/main/main.cpp

HEADERS += \
    src/main/kinematics_solver.h
