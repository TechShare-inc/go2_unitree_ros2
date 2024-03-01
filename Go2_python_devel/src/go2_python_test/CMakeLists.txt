cmake_minimum_required(VERSION 3.5)
project(go2_python_test)

# 依存関係を見つける
find_package(ament_cmake REQUIRED)

# Pythonスクリプトのインストール
install(PROGRAMS
  sport_mode_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# その他のament設定
ament_package()
