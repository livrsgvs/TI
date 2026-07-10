"""
全局调试开关 (debug_config.py)
==============================
所有模块共用此标志控制 print 输出。
main.py 在初始化前根据 Config.DEBUG_MODE 设置此值。

用法：
    import debug_config
    if debug_config.DEBUG:
        print("...")
"""

DEBUG = True
