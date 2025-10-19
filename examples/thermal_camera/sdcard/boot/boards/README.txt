
Board config files, first line MUST board id !!!

板子配置文件，第一行必须是板子 id ！！！

Keywords:

- id:
	Unique id

- name:
	Board name

- panel:
	LCD panel id, refer to https://wiki.sipeed.com/maixpy/doc/zh/vision/display.html

- cam_mclk:
	Camera MCLK pin number, according to hardware connection.

- cam_flip:
	Vertical flip camera.

- cam_mirror:
	Horizontally mirror camera.

- disp_flip:
	Display(LCD) show Vertical flip.

- disp_mirror:
	Display(LCD), Horizontally mirror.

- disp_max_backlight:
	Display(LCD) backlight max power, to limit max brightness and power, user set in API(`disp.set_backlight(value)`) actually set to `value * disp_max_backlight`.





