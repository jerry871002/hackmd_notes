---
tags: A Tour of C++, C++
---

# 3 模組化

## 翻譯單位（translation unit）

> A **translation unit** is the basic unit of compilation in C++. It consists of the contents of a single source file, plus the contents of any header files directly or indirectly included by it, minus those lines that were ignored using conditional preprocessing statements.

:::info
這個觀念跟他介紹的一個 C++ 20 的新功能「模組（Modules）」有關，不過因為這個還沒有多少人實作也不普及，就先跳過囉
:::

## 錯誤處理

### 例外

標準程式庫的 `<stdexcept>` 可以去了解一下，裡面有很多定義好的例外，例如
```cpp
double& Vector::operator[](int i) {
    if (i < 0 || size() <= i)
        throw out_of_range{"Vector::operator[]"};
    return elem[i];
}
```
處理的方式像這樣
```cpp
void f(Vector& v) {
    // ...
    try { // exceptions here are handled by the handler defined below
        v[v.size()] = 7; // try to access beyond the end of v
    } catch (out_of_range& err) { // oops: out_of_range error
        // ... handle range error ...
        cerr << err.what() << endl;
    }
    // ...
}
```
有些函數永遠不應該丟出例外，可以宣告為 `noexcept`
```cpp
void user(int sz) noexcept {
    Vector v(sz);
    iota(&v[0], &v[sz], 1); // fill with 1, 2, 3, 4...
}
```
如果這種函數基於某種原因還是丟出例外的話，C++ 會呼叫 `std::terminate()` 立即中止程式

### 斷言

有分成可以在執行期判斷的 `assert` 跟在編譯期檢查的 `static_assert`

```cpp
void f(const char* p) {
    assert(p != nullptr)
}
```
`static_assert` 只能用在可以用**常數運算式**表達的東西上，[第一章](/UkLiREP7Q_mKgI6Pun58YQ)有介紹常數運算式
```cpp
constexpr double C = 299792.458; // km/s

void f(double speed) {
    const double local_max = 160.0 / (60 * 60);         // km/h to km/s
    static_assert(speed < C, "can't go that fast");     // error : speed must be a constant
    static_assert(local_max < C, "can't go that fast"); // OK
    // ...
}
```
`static_assert(A, S)` 的 `S` 不一定要寫，不寫的話編譯器會印出預設訊息

:::info
Notes：
1. 這章一直寫到錯誤處理比較好的方式是 RAII (Resource Acquisition Is Initialization)，之後應該就會看到了
2. 之前寫 C++ 檢查錯誤就是用一堆 `if` 跟 `cout`，看來該好好學一下正規的方式了
:::

## 預設函數引數（default function argument）

雖然之前就可以用**多載**的方式來達成
```cpp
void print(int value, int base);

void print(int value) {
    print(value, 10);
}
```

但這樣做更簡單
```cpp
void print(int value, int base=10);

print(x, 16);
print(x, 60);
print(x);
```

:::info
Notes：這個功能超好用！`Python` 裡面也有這樣的功能，之前上作業系統的時候有些 system call 參數超多但大部分都有一個常用的值，有這個功能的話就不用每次都寫得落落長了
:::

## 結構連結（structured binding）

```cpp
struct Entry {
    std::string name;
    int value;
};

Entry read_entry(std::istream& is) {
    std::string s;
    int i;
    is >> s >> i;
    return {s, i}; // C++ 11
}
```

```cpp
auto [n, v] = read_entry(std::cin); // C++ 17
std::cout << "{" << n << ", " << v << "}\n";
```

不只能用在函式的回傳值喔，結合迴圈使用也很厲害
```cpp
map<string, int> m;
// fill m
for(const auto [key, value]: m)
    cout << "{" << key << ", " << value << "}\n";
```


:::info
Notes：這也是我在 `Python` 裡面常用的功能，**一次回傳多個值**跟**一次指定多個值**。雖然還是要建一個 `struct` 才能達成，但兩個新功能已經讓這件事比之前好用很多了
:::

## Reference
* [What is a “translation unit” in C++](https://stackoverflow.com/questions/1106149/what-is-a-translation-unit-in-c)，Stack Overflow
* [<stdexcept>](http://www.cplusplus.com/reference/stdexcept/)，cplusplus.com - The C++ Resources Network