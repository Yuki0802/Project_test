// 运行该程序前，必须下载绘图库。
// 详见：http://hi.baidu.com/yangw80/blog/item/63ff598072a9f9d09023d97f.html
#include "graphics.h"
#include "time.h"
#include "conio.h"

void main()
{
	// init randomize seed
	srand((unsigned) time(NULL));

	// init graph mode
	initgraph(640, 480);

	int x,y;
	char s[2] = {0};

	while(!kbhit())
	{
		for (int i=0; i<479; i++)
		{
			setcolor(RED);
			for (int j=0; j<3; j++)
			{
				x = (rand() % 80) * 8;
				y = (rand() % 20) * 24;

				s[0] = (rand() % 26) + 65;
				outtextxy(x,y,s);
			}

			setcolor(0);
			line(0,i,639,i);

			Sleep(10);
			if (kbhit()) break;
		}
	}

	closegraph();
}
