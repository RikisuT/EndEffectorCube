idf_component_register(SRCS "EndEffectorCube.cpp" "RPYSolver.c"
                               "../repo/Fusion/Fusion/FusionAhrs.c"
                               "../repo/Fusion/Fusion/FusionBias.c"
                               "../repo/Fusion/Fusion/FusionCompass.c"
                    INCLUDE_DIRS "."
                                 "../repo/Fusion/Fusion")
