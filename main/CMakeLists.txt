idf_component_register(SRCS main.c timeruse.c control.c servo.c IMU.c icm426xxSPI.c icm426xxI2C.c devicectl.c webserver.c web_ota.c web_api.c web_handler.c taskwatch.c
    EMBED_FILES root.html setup.html setup2.html ota.html monitor.html favicon.ico connecttest.txt)

# 浮動小数点リテラルを float として扱わせる
target_compile_options(${COMPONENT_LIB} PRIVATE -fsingle-precision-constant)

# double への暗黙昇格を警告・エラーにする
# target_compile_options(${COMPONENT_LIB} PRIVATE -Wdouble-promotion -Werror=double-promotion)
