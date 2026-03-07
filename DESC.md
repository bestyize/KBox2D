使用kmp-transfer-skill 这个skill

# 工程背景

本工程是一个compose跨平台项目，支持Android、iOS、desktop，目的是复刻经典的物理引擎jbox2d。
因为jbox2d是使用Java写的，因此无法在compose跨平台项目中使用。

# 工程目标

实现与jbox2d相同的能力，不依赖jvm，最好接口与jbox2d兼容。只把包名改为xyz.thewind.kbox2d。

在当前工程中新建一个kbox2d模块，存放新代码。不需要迁移jbox2d所有的模块，只需要实现jbox2d-library完整功能即可

# 参考文献

jbox2d官方地址：https://github.com/jbox2d/jbox2d


# 实施计划



根据以上背景，帮我制定详细的实施计划，保存为PLAN.md,然后实施迁移