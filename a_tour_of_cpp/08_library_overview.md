---
tags: A Tour of C++, C++
---

# 8 程式庫概觀

[![hackmd-github-sync-badge](https://hackmd.io/r5-gYE22SmKeGoXJQMJT4Q/badge)](https://hackmd.io/r5-gYE22SmKeGoXJQMJT4Q)


這章只有寫一些介紹而已，我就挑幾個我想記得的點來講就好（大部分是從忠告這個 section 來的）

* 不要重新發明輪子，使用程式庫
* 如果可以選擇，用標準程式庫而非其他程式庫
* 標準程式庫的東西都定義在命名空間 `std` 裡面

## 有關 `std` 命名空間

C++ 裡面也有提供 C 的標準程式庫，不過名字稍微不一樣，例如 C 的 `stdio.h` 會變成 `cstdio`。除了名字不一樣之外，C++ 版本的還會把東西的宣告放進 `std` 命名空間裡，可以看看底下四個範例，第四個會出錯喔！

1. 
    ```cpp
    #include<cstdio>

    int main(int argc, char const *argv[]) {
        printf("Hello world\n"); 
        return 0;
    }
    ```
2. 
    ```cpp
    #include<stdio.h>

    int main(int argc, char const *argv[]) {
        printf("Hello world\n"); 
        return 0;
    }
    ```
3. 
    ```cpp
    #include<cstdio>

    int main(int argc, char const *argv[]) {
        std::printf("Hello world\n"); 
        return 0;
    }
    ```
4. 
    ```cpp
    #include<stdio.h>

    int main(int argc, char const *argv[]) {
        std::printf("Hello world\n"); // error: no member named 'printf' in namespace 'std'
        return 0;
    }
    ```

:::info
可能有人會想為什麼第一個也是正確的，根據[維基百科](https://zh.wikipedia.org/wiki/C%2B%2B%E6%A8%99%E6%BA%96%E5%87%BD%E5%BC%8F%E5%BA%AB)表示，雖然 C++ 版本的應該要在 `std` 命名空間裡面，但很少編譯器嚴格遵守，通常的做法是**同時放在全域與 `std` 內**，例如 `printf` 和 `std::printf` 兩者均有
:::