cmake_minimum_required(VERSION 2.6)

FIND_PACKAGE(ITK REQUIRED)
INCLUDE(${ITK_USE_FILE})

PROJECT(PriorityBeliefPropagation)

ADD_EXECUTABLE(TestGrayscaleInpainting TestGrayscaleInpainting.cxx
Message.cxx MessageVector.cxx Comparisons.cxx
GrayscaleInpainting.cxx Node.cxx

Helpers.cxx  Debugging.cxx)
TARGET_LINK_LIBRARIES(TestGrayscaleInpainting ITKCommon ITKIO ITKBasicFilters)
