// 运行该程序前，必须下载绘图库。
// 详见：http://hi.baidu.com/yangw80/blog/item/63ff598072a9f9d09023d97f.html
#include <graphics.h>
#include <time.h>
#include <conio.h>

#define MAXSTAR 200	// 星星总数

struct STAR
{
	double x;
	int y;
	double step;
	int color;
};

STAR star[MAXSTAR];

// 初始化星星
void InitStar(int i)
{
	star[i].x = 0;
	star[i].y = rand() % 480;
	star[i].step = (rand() % 5000) / 1000.0 + 1;
	star[i].color = (int)(star[i].step * 255 / 6.0 + 0.5);	// 速度越快，颜色越亮
	star[i].color = RGB(star[i].color, star[i].color, star[i].color);
}

// 移动星星
void MoveStar(int i)
{
	// 擦掉原来的星星
	putpixel((int)star[i].x, star[i].y, 0);

	// 计算新位置
	star[i].x += star[i].step;
	if (star[i].x > 640)	InitStar(i);

	// 画新星星
	putpixel((int)star[i].x, star[i].y, star[i].color);
}

// 主函数
void main()
{
	srand((unsigned)time(NULL)); // 随机种子
	initgraph(640, 480);	// 打开图形窗口

	// 初始化所有星星
	for(int i=0; i<MAXSTAR; i++)
	{
		InitStar(i);
		star[i].x = rand() % 640;
	}

	// 绘制星空，按任意键退出
	while(!kbhit())
	{
		for(int i=0; i<MAXSTAR; i++)
			MoveStar(i);
		Sleep(20);
	}

	closegraph();    // 关闭图形窗口
}
