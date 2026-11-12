idf_component_register(SRCS main.c timeruse.c control.c servo.c IMU.c icm426xxSPI.c icm426xxI2C.c devicectl.c webserver.c web_ota.c web_api.c web_handler.c taskwatch.c
    EMBED_FILES root.html setup.html setup2.html ota.html monitor.html favicon.ico connecttest.txt)

# Treat floating-point literals as floats.
target_compile_options(${COMPONENT_LIB} PRIVATE -fsingle-precision-constant)

# Warn or fail on implicit promotion to double.
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wdouble-promotion -Werror=double-promotion)
