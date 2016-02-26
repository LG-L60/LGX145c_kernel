# Исходники kernel LG L60 X145 v10c

Дебажный вариант kernel собирать командой

./mk lo1 n k


Релизный вариант kernel собирать командой

./mk -o=TARGET_BUILD_VARIANT=user lo1 new kernel


Вариант kernel для рекавери собирать командой

./mk -o=TARGET_BUILD_VARIANT=recovery lo1 new kernel

