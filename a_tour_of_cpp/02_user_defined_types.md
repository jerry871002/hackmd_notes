---
tags: A Tour of C++, C++
---

# 2 使用者定義型別

## 類別

裡面有一句話我覺得很有趣（應該說之前不知道）

> `struct` 和 `class` 基本上沒什麼不同，`struct` 只是成員都預設為 `public` 的一種 `class`

所以說 `struct` 也可以定義建構子和成員函式

## `union` & `varion`

之前學 C++ 的時候覺得 `union` 不太好用，因為容易出錯又要多寫東西來判斷型別，這邊介紹標準程式庫裡的 `variant` 來替代 `union`

```cpp
struct Entry {
    string name;
    variant<Node*, int> v; // v 不是 Node* 就是 int
};

void f(Entry* pe) {
    if (holds_alternative<int>(pe->v)) // v 是一個 int 嗎？
        cout << get<int>(pe->v);       // 取得那個 int
    // do something else...
}
```

## 列舉

這邊講的是 `enum class` 而不是之前比較熟悉的「普通的」`enum` 喔。`enum class` 跟「普通的」`enum` 不一樣的地方在於，「普通的」`enum` 可以把自己的值轉換為整數，`enum class` 則不行

* `enum class` 用法像這樣
    ```cpp
    enum class Color { red, blue, green };
    enum class Traffic_light { green, yellow, red };

    Color col = Color::red;
    Traffic_light light = Traffic_light::red;

    Color x = red; // 錯誤！沒講講清楚是哪個 red
    ```
* 「普通的」`enum` 用法則像這樣
    ```cpp
    enum Color { red, blue, green }; // 「普通的」enum
    int col = green; // col 的值是 2
    ```
C++ 傳統的 `enum` 主要有三個問題：

* 預設可以被自動轉換成 int，在某些不應該被轉換成 int 的情況下，可能會造成問題
* 所列舉項目由於沒有特殊的範圍（scope），所以很容易造成衝突
* 沒辦法指定列舉型別底層的類型，容易造成混淆、不相容、同時也沒辦法做 forward declaration（這個我們這邊沒討論到，可以去 Reference 看原網頁）

`enum class` 預設只定義了**指定**、**初始化**和**比較**，用 `enum class` 可以更明確表達自己的意圖，就如同這章最後一部分所說
> 使用 `enum class` 而不是「平常的」`enum` 以減少意外狀況


:::info
Notes：
1. `enum class` 是 C++ 11 的功能
2. 這章強調的大多是如何更**明確地表達「意圖」**，這和之前印象中 C++ 常常在用一些隱晦的 trick 大不相同（看來想法要跟上時代呢）
    > 當內建型別過於低階時，最好使用定義嚴謹的使用者型別而不是內建型別（2.1節）
:::

## Reference
* [使用 enum class 取代傳統的 enum](https://kheresy.wordpress.com/2019/03/27/using-enum-class/)，Heresy's Space