---
tags: C++
---

# 1 基本概念

[![hackmd-github-sync-badge](https://hackmd.io/UkLiREP7Q_mKgI6Pun58YQ/badge)](https://hackmd.io/UkLiREP7Q_mKgI6Pun58YQ)


## 變數

很長的數字可以用單引號讓他比較好閱讀，例如 $\pi$ 的數值可以寫成

* `3.14159'26535'89793'23846'26433'83279'50288`
* `0x3.243F'6A88'85A3'08D3`

:::info
Notes：雖然用到的機會感覺不大，但知道一下也無仿
:::

## 初始化

以下三者意義相同

```cpp
double d1 = 2.3;
double d2 {2.3};
double d3 = {2.3};
```

> 使用 `{...}` 時 `=` 可以寫也可以不用寫

使用串列形式的 `{...}` 可以讓初始化更有彈性，例如
```cpp
complex<double> z1 = 1;
complex<double> z2 {d1, d2};
complex<double> z3 = {d1, d2};

vector<int> v {1, 2, 3, 4, 5, 6};
```

`=` 是跟 `C` 類似的傳統形式，串列形式的 `{...}`除了更有彈性外，也可以避免會遺失資訊的轉換

```cpp
int i1 = 7.8; // i1 的值會變成 7
int i2 {7.8}; // 會顯示錯誤資訊：floating-point to integer conversion
```

用 `auto` 初始化時通常會用 `=`，因為沒有型別轉換的問題，但要用 `{...}` 也是沒問題的，例如

```cpp
auto b1 = true;   // a bool
auto ch = 'x';    // a char
auto i = 123;     // an int
auto d = 1.2;     // a double
auto z = sqrt(y); // z has the type of whatever sqrt(y) returns
auto b2 {true};   // a bool
```

:::info
Notes：`{...}` 是新學到的部分，可以避免型別轉換的問題感覺可以讓生活更簡單快樂一些，應該試著用用看
:::

## 常數

有兩種符號來表達，但意義有些微的不同

* `const`：代表「不會改變這個值」，可以執行時再計算值
* `constexpr`：代表「編譯時賦值」，值必須由編譯器計算

```cpp
double sum(const vector<double>&); // sum will not modify its argument

vector<double> v {1.2, 3.4, 4.5};  // v is not a constant
const double s1 = sum(v);          // OK: evaluated at run time
constexpr double s2 = sum(v);      // error : sum(v) not constant expression
```

這邊要解釋一下**常數運算式**（constant expression），除了被定義成常數的變數外，函數也可以透過關鍵字 `constexpr` 成為常數運算式，例如

```cpp
constexpr double square(double x) { return x * x; } 

const int dmv = 17; // dmv is a named constant
int var = 17;       // var is not a constant

constexpr double max1 = 1.4 * square(dmv); // OK since square(17) is a constant expression
constexpr double max2 = 1.4 * square(var); // error: constexpr variable 'max2' must be initialized by a constant expression
                                           // 因為 var 不是 constant expression
const double max3 = 1.4 * square(var);     // OK, may be evaluated at run time
```

函數要成為 `constexpr` 有一些限制跟要注意的地方
* 不能更動非區域變數的值
* 可以傳非常數的 arguments 進去，不過這樣函式 return 回來的東西就不是 `constexpr` 了
* `constexpr` 函數要盡量簡單，不過迴圈跟區域變數等都是可以合法使用的（只要能在編譯期算得出來）

例如底下也是一個合法的 `constexpr` 函數
```cpp
constexpr double nth(double x, int n) {
    double res = 1;
    int i = 0;
    
    while (i < n) {
        res *= x;
        i++;
    }
    
    return res;
}
```

會用到 constant expressions 的地方
* 陣列的範圍（大小）
* case 的標籤
* 樣板值引數（template value arguments）
* 使用 `constexpr` 宣告的常數
> 樣板值引數我也還不太清楚，之後應該會學到

:::info
Notes：
1. `constexpr` 是 C++ 11 的新功能，編譯時記得要加 `--std=c++11`
2. `constexpr` 函數裡面如果想用`if`、`for`、`while` 等流程控制或區域變數，這是 C++ 14 才允許的，編譯時要加 `--std=c++14`
3. 書上說後面會講到 `constexpr` 在 `enum` 宣告和 `switch` 邏輯判斷分支上的應用，可以期待一下
4. 另外在測試的時候有發現 `constexpr function` 一定要在使用前就定義（就是不能只 declare 不 define 啦）。[Stack Overflow 上是這麼解釋的](https://stackoverflow.com/questions/24152418/declaring-constexpr-functions-or-methods)
    > constant expressions require the definition to be **before** the constant expression
:::

## 簡化的 `for`

```cpp
void print() {
    int v[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    for (auto x: v)
        std::cout << x << '\n'; 

    for (auto x: {10, 21, 32, 43, 54, 65})
        std::cout << x << '\n';
}
```

:::info
Notes：
1. `auto` 配上 range-based 的迴圈，感覺快跟 `Python` 的迴圈一樣了
2. 也是 C++ 11 的新功能，編譯時記得要加 `--std=c++11`
:::

## Null 指標

當指標沒有指向任何物件時，會將指標設成 `nullptr`

```cpp
double* pd = nullptr;
Link<Record>* lst = nullptr; // pointer to a Link to a Record
int x = nullptr; // error : nullptr is a pointer not an integer
```

因為 `NULL` 實際上代表的是整數 0，用 `nullptr` 而不是 `NULL` 的好處是可以更清楚的表達「空指標」這件事（而不是整數 0），避免混淆也避免錯誤

:::info
Notes：有點像 `Python` 裡面有個 `None` 來表達「空」這件事呢
:::

## 更簡潔的條件判斷

下面兩種寫法是在做一模一樣的事情

```cpp
void do_something(vector<int>& v) {
    if (auto n = v.size(); n != 0) {
        // do something here
    }
}
```

```cpp
void do_something(vector<int>& v) {
    if (auto n = v.size()) {
        // do something here
    }
}
```

原因就是 assignment expression 也有回傳值，[C11 Standard](http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1570.pdf) 裡面是這樣寫的
> An assignment expression has the value of the left operand after the assignment


:::info
Notes：新的 `Python 3.8` 也用 `:=`（the walrus operator）來達成這樣的效果，沒想到 `C/C++` 裡面早就有了（應該是我知道的太少）
:::

## 名詞比較

因為太久沒碰 `C++` 了（或是說太久沒認真研究寫程式這件事了），有些之前學過的名詞快要忘記他們的定義了，害我寫東西的時候很卡，趕快把它們寫下來以防之後又忘記

### argument & parameter

根據 [Stack Overflow 上的回答](https://stackoverflow.com/questions/1788923/parameter-vs-argument/1788926#1788926)

* **引數 (Argument)** 用於呼叫函式
    An **argument** is an expression used when calling the method
* **參數 (Parameter)** 則用於函式的簽章 (函式的宣告)
    A **parameter** is the variable which is part of the method’s signature (method declaration)
    
```cpp
void Foo(int i, float f) {
    // Do things
}

void Bar() {
    int anInt = 1;
    Foo(anInt, 2.0);
}
```
`i` 跟 `j` 是參數 (Parameter)，`anInt` 跟 `2.0` 則是引數 (Argument)

### statement & expression

* **陳述句（Statement）**
    程式碼的單位，這段程式碼不會產生一個值
* **表達式、表示式（Expression）**
    程式碼的單位，這段程式碼最終會產生（回傳）一個值，而這個值不一定會被開發者賦予變數
    
:::info
Notes：看來重點就是會不會有回傳值
:::

### definition & declaration

* **宣告（Declaration）**
    讓 Compiler 知道東西的 name 和 type

    ```cpp
    int Foo(int a, int b);
    class CFoo;
    extern int a;
    ```

* **定義（Definition）**
    告訴 Compiler 所有 create 這個東西所需要的 information (e.g. function body, class fields and methods)，當然，當你 define 一個東西時也會一併 declare 它

    ```cpp
    int Foo(int a, int b)  {
        return a + b;
    }

    class CFoo {
    public:
        void CFoo(int data) : m_data(data) {}
    private:
        int m_data;
    };

    int a; //declaration and definition
    ```

## Reference
* [引數 (Argument) vs. 參數 (Parameter)](https://notfalse.net/6/arg-vs-param)，NotFalse 技術客
* [“Parameter” vs “Argument”](https://stackoverflow.com/questions/1788923/parameter-vs-argument/1788926#1788926)，Stack Overflow
* [Declaration and Definition in C++](https://coherence0815.wordpress.com/2015/04/30/declaration-and-definition-in-c/)，打字猴
* [Declaring constexpr functions or methods](https://stackoverflow.com/questions/24152418/declaring-constexpr-functions-or-methods)，Stack Overflow
* [What does an assignment return?](https://stackoverflow.com/questions/9514569/what-does-an-assignment-return)，Stack Overflow
* [潮．C++11 | constexpr, constexpr function](https://medium.com/@tjsw/%E6%BD%AE-c-constexpr-ac1bb2bdc5e2)，Medium
* [What’s New In Python 3.8](https://docs.python.org/3/whatsnew/3.8.html)，Python 3.8.5 documentation
*  [Writing Mathematic Fomulars in Markdown](https://csrgxtu.github.io/2015/03/20/Writing-Mathematic-Fomulars-in-Markdown/)，Archer Reilly
*  [Day16 函式陳述句與函式表示式](https://ithelp.ithome.com.tw/articles/10192146?sc=pt)，JavaScript基礎二三事，iT邦幫忙