navigation_app:循环导航目标点插件
功能：用户可在该插件中同时添加多个导航目标点，类似于2d_nav_gal。
用法：(1)运行导航功能包后，运行rosrun navigation_app get_points。
      (2)在RVIZ中的菜单栏依次点击panels->add new panels->NavApp,然后就可以看见左边新增了一个操作界面
      (3)界面内容：List Name:导航点列表的名字; Loop Times:循环次数; Current Pose:当前位姿; Nav Station:导航目标点位姿队列。
      (4)点击Add，依次输入导航点的x,y,theta的值，点击OK即可完成一个导航点的设置。点击列表相应行，再点击Remove即可删除点。导航点列表设置完成后，点击
         start即可开始循环导航。点击stop停止。
      (5)在导航过程中也可正常使用2D Nav Goal,同时也可以使用stop来停止2D Nav Goal产生的机器人的移动。

注意：应将从github上下载的包里面的所有的功能包全部直接放在工作空间的src/文件夹下，而不是src/bobac2/下，或是src/bobac2_masters/下。
      如果想要在RVIZ中看到自己所设的导航点在哪里，可在display中添加MarkerArray。
