{
  "targets": [
    {
      "target_name": "decode_emitter",
      "sources": [
        "src/binding.cc",
        "src/common_func.cpp",
        "src/decode-emitter.cc",
        "src/decoder/common.c",
        "src/decoder/gnss_math.c",
        "src/decoder/mixed_raw.c",
        "src/decoder/model.c",
        "src/decoder/rtcm.c",
        "src/decoder/rtkcmn.c",
        "src/decoder/beidou.cpp",
        "src/decoder/decode_interface.cpp",
        "src/decoder/E2E_protocol.cpp",
        "src/decoder/imu_raw.cpp",
        "src/decoder/ins_save_parse.cpp",
        "src/decoder/ins401.cpp",
        "src/decoder/ins401c.cpp",
        "src/decoder/kml.cpp",
        "src/decoder/NPOS122_decoder.cpp",
        "src/decoder/openrtk_user.cpp",
        "src/decoder/rtcm_split.cpp",
        "src/decoder/rtk330la_decoder.cpp",
        "src/decoder/SplitByTime.cpp"
      ],
      'cflags!': [ '-fno-exceptions'],
      'cflags_cc!': [ '-fno-exceptions'],
      'include_dirs': [
        "<!@(node -p \"require('node-addon-api').include\")",
        "src/decoder/"],
      'dependencies': ["<!(node -p \"require('node-addon-api').gyp\")"],
      'defines': [ 'OUTPUT_INNER_FILE' ],
      'conditions': [
        ['OS=="win"', {
          "msvs_settings": {
            "VCCLCompilerTool": {
              "ExceptionHandling": 1
            }
          }
        }],
        ['OS=="mac"', {
          "xcode_settings": {
            "CLANG_CXX_LIBRARY": "libc++",
            'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
            'MACOSX_DEPLOYMENT_TARGET': '10.7'
          }
        }]
      ]
    }
  ]
}
