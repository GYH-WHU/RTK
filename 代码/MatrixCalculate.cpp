#include"RTK_Structs.h"
bool MatrixMultiply(int m1, int n1, int m2, int n2, const double M1[], const double M2[], double M3[])
{
    // 首先判断矩阵相乘的大小是否合法
    if (m1 <= 0 || n1 <= 0 || m2 <= 0 || n2 <= 0 || n1 != m2)
    {
        printf("Error dimension in MatrixMultiply!\n");
        return false; // 矩阵大小不合法，无法进行矩阵相乘
    }

    // 执行矩阵相乘运算
    for (int i = 0; i < m1; i++)
    {
        for (int j = 0; j < n2; j++)
        {
            M3[i * n2 + j] = 0; // 初始化结果矩阵元素为0
            for (int k = 0; k < n1; k++)
            {
                M3[i * n2 + j] = M1[i * n1 + k] * M2[k * n2 + j] + M3[i * n2 + j]; // 矩阵相乘
            }
        }
    }

    return true;
}

int MatrixInv(int n, double a[], double b[])
{
    int i, j, k, l, u, v, is[200], js[200];
    double d, p;

    if (n <= 0)
    {
        printf("Error dimension in MatrixInv!\n");
        return 0;
    }

    // 将输入矩阵赋值给输出矩阵b，下面对b矩阵求逆，a矩阵不变
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            b[i * n + j] = a[i * n + j];
        }
    }

    for (k = 0; k < n; k++)
    {
        d = 0.0;
        for (i = k; i < n; i++)   // 查找右下角方阵中主元素的位置 
        {
            for (j = k; j < n; j++)
            {
                l = n * i + j;
                p = fabs(b[l]);
                if (p > d)
                {
                    d = p;
                    is[k] = i;
                    js[k] = j;
                }
            }
        }

        if (d < DBL_EPSILON)   // 主元素接近于0，矩阵不可逆
        {
            printf("Divided by 0 in MatrixInv!\n");
            return 0;
        }

        if (is[k] != k)  // 对主元素所在的行与右下角方阵的首行进行调换
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = is[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        if (js[k] != k)  // 对主元素所在的列与右下角方阵的首列进行调换
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = i * n + js[k];
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }

        l = k * n + k;
        b[l] = 1.0 / b[l];  // 初等行变换
        for (j = 0; j < n; j++)
        {
            if (j != k)
            {
                u = k * n + j;
                b[u] = b[u] * b[l];
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                for (j = 0; j < n; j++)
                {
                    if (j != k)
                    {
                        u = i * n + j;
                        b[u] = b[u] - b[i * n + k] * b[k * n + j];
                    }
                }
            }
        }
        for (i = 0; i < n; i++)
        {
            if (i != k)
            {
                u = i * n + k;
                b[u] = -b[u] * b[l];
            }
        }
    }

    for (k = n - 1; k >= 0; k--)  // 将上面的行列调换重新恢复
    {
        if (js[k] != k)
        {
            for (j = 0; j < n; j++)
            {
                u = k * n + j;
                v = js[k] * n + j;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
        if (is[k] != k)
        {
            for (i = 0; i < n; i++)
            {
                u = i * n + k;
                v = is[k] + i * n;
                p = b[u];
                b[u] = b[v];
                b[v] = p;
            }
        }
    }

    return (1);
}