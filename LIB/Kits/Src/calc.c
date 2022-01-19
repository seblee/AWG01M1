#include "stdint.h"
#include "calc.h"
#include "global.h"
#include "sys_conf.h"

int16_t lim_min_max(int16_t min, int16_t max, int16_t data)
{
    if (data <= min)
    {
        return min;
    }
    else if (data >= max)
    {
        return max;
    }
    else
    {
        return data;
    }
}

int16_t bin_search(uint16_t *a_ptr, uint16_t array_size, uint16_t key)
{
    int low = 0, high = array_size - 1, mid;

    if ((key < *(a_ptr)) || (key > *(a_ptr + array_size)))
    {
        return -1;
    }

    while (low <= high)
    {
        mid = (low + high) / 2;

        if ((*(a_ptr + mid) <= key) && (*(a_ptr + mid + 1) > key) && ((mid + 1) < array_size))
        {
            return mid;
        }
        else
        {
            if (*(a_ptr + mid) > key)
            {
                high = mid - 1;
            }
            else
            {
                low = mid + 1;
            }
        }
    }
    return -1;
}

void quick(uint16_t *a, int16_t i, int16_t j)  //快速排序
{
    int16_t m, n, temp;
    uint16_t k;
    m = i;
    n = j;
    k = a[(i + j) / 2]; /*选取的参照*/
    do
    {
        while (a[m] < k && m < j)
        {
            m++; /* 从左到右找比k大的元素*/
        }
        while (a[n] > k && n > i)
        {
            n--; /* 从右到左找比k小的元素*/
        }
        if (m <= n) /*若找到且满足条件，则交换*/
        {
            temp = a[m];
            a[m] = a[n];
            a[n] = temp;
            m++;
            n--;
        }
    } while (m <= n);
    if (m < j)
    {
        quick(a, m, j); /*运用递归*/
    }
    if (n > i)
    {
        quick(a, i, n);
    }
}

/**
 *  中位值平均滤波
 * @param pData:没有滤波的数据
 * @param nSize:数据大小
 * @return:滤波数值
 */
unsigned short MedianFilter(unsigned short *pData, int nSize)
{
    unsigned short max, min;
    int sum;
    int ave;
    int i;
    if (nSize > 2)
    {
        max = pData[0];
        min = max;
        sum = 0;
        for (i = 0; i < nSize; i++)
        {
            sum += pData[i];
            if (pData[i] > max)
            {
                max = pData[i];  //一个循环之后max就是最大的值
            }

            if (pData[i] < min)
            {
                min = pData[i];  //一个循环之后min就是最小的值
            }
            //						rt_kprintf("pData[i] = %d\n", pData[i] );
        }

        sum = sum - max - min;  //去掉最大的值和最小的值
        ave = sum / (nSize - 2);
        //				rt_kprintf("sum = %d,ave = %d,u16Ain[i] = %d\n", sum,ave );
        return ave;  //对N-2个数求平均值
    }

    return 0;
}
//冒泡法
void bubble_sort(int16_t *a, uint8_t n)
{
    uint8_t i, j;
    for (i = 0; i < n; i++)
    {
        for (j = i; j < n; j++)
        {
            if (*(a + i) > *(a + j))
            {
                int temp;
                temp     = *(a + i);
                *(a + i) = *(a + j);
                *(a + j) = temp;
            }
        }
    }
}
