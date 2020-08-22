---
tags: A Tour of C++, C++
---

# 4 類別

[![hackmd-github-sync-badge](https://hackmd.io/ZGmD42D1SyqazNbv7Yaa4Q/badge)](https://hackmd.io/ZGmD42D1SyqazNbv7Yaa4Q)


這個章節剛好可以拿來複習之前物件導向學的東西

## 運算子多載（operator overloading）

### 可以被多載的 operator

* Arithmetic  
    `+`, `-`, `*`, `/`,` %`
* Bitwise  
    `^`(XOR), `&`, `|`, `~`(NOT), `>>`, `<<`
* Correlational  
    `<`, `<=`, `>`, `>=`, `!=`, `==`
* Logic  
    `!`, `&&`, `||`
* Assignment  
    `=`, `+=`, `-=`, `*=`, `/=`, `%=`, `<<=`, `>>=`, `&=`, `^=`, `|=`
* Other  
    `++`, `--`, `[]`, `()`, `->`, `new`, `new []`, `delete`, `delete []`
    
### 成員函數（member function） vs 全域函數（global function）

#### Member function

* 通常使用 **`this`** 來操作運算的 left operand（不用額外寫在參數裡面的意思）
    > use the `this` pointer implicitly to obtain one of their class object arguments (the left operand for binary operators).
* `[]`, `()`, `->` **一定**要宣告成 member function

#### Global function

* Binary operator 的兩個參數都要清楚寫下來
* 通常會宣告為該類別的 **`friend`**（才能取用 `private` 的東西）
* 如果 left operand **不屬於該類別時**，**一定**要宣告成 global function（像是 `<<` 跟 `>>`，是輸入輸出那個喔，不是 bitwise 那個）

#### 很長的範例

```cpp
class Complex {
    double re, im;
public:
    // constructors
    Complex(double r, double i): re{r}, im{i} {}
    Complex(double r): re{r}, im{0} {}
    Complex(): re{0}, im{0} {}

    double real() const { return re; }
    void real(double d) { re = d; }

    double image() const { return im; }
    void image(double d) { im = d; }

    // member function
    Complex& operator+=(Complex z) {
        re += z.re;
        im += z.im;
        return *this;
    }

    Complex& operator-=(Complex z) {
        re -= z.re;
        im -= z.im;
        return *this;
    }

    // not member function but friend function
    friend ostream& operator<<(ostream& out, const Complex& a);
};

// define outside the class (global function)
Complex operator+(Complex a, Complex b) { return a += b; }
Complex operator-(Complex a, Complex b) { return a -= b; }
Complex operator-(Complex a) { return {-a.real(), -a.image()}; } // negative

bool operator==(Complex a, Complex b) {
    return a.real() == b.real() && a.image() == b.image();
}

bool operator!=(Complex a, Complex b) {
    return !(a == b);
}

ostream& operator<<(ostream& out, const Complex& a) {
    out << a.re << " + " << a.im << "i";
    return out;
}
```

#### 另一個範例

這個範例更詳細解釋 member function 跟 global funciton 的使用時機（以下 `c` 代表 `Complex`，`i` 代表 `int`）

如果想使用 `c3 = c1 + i`，使用 member function 就可以了

```cpp
Complex Complex::operator+(int& i) { return {real + i, imag}; }
```

但如果想使用 `c3 = i + c1`，就要使用 global function（如果要取用內部 `private` 資料還要宣告 `friend`）

```cpp
Complex operator+(int& i, Complex& c) { return {i + c.real, c.imag}; }
```

:::info
Notes：
1. 其實這整段都不是書上的內容，只是因為我忘光光了再拿出來複習而已
2. 感謝物件導向的講義寫的很清楚
:::

## 好用的 `initializer_list`

如果定義了一個**容器**（**container**）物件的話，想要初始化他們必須要一個一個設定，但 C++ 11 提供了一個方便的初始化機制
> 容器就是**保存一群元素**的意思

```cpp
#include<initializer_list>
#include<algorithm> // copy()

class Vector {
public:
    Vector(int s): elem{new double[s]}, sz{s} {
        for (int i = 0; i != s; ++i)
            elem[i] =  0;
    }

    Vector(std::initializer_list<double>);

    ~Vector() { delete[] elem; }

private:
    double* elem;
    int sz;
};

Vector::Vector(std::initializer_list<double> lst)
    :elem{new double[lst.size()]}, sz{static_cast<int>(lst.size())} {
    std::copy(lst.begin(), lst.end(), elem);
}
```
如此一來就可以用底下的方式初始化
```cpp
Vector v1 = {1, 2, 3, 4, 5};
Vector v2 = {1.23, 3.45, 6.7, 8};
```

:::info
Notes：很方便的新功能！（語無倫次）
:::

## 抽象類別（abstract class）

### `virtual`

在定義抽象類別前先看看 `virtual` 這個關鍵字

定義成 `virtual` 的函數叫做**虛擬函數**（**virtual function**），在我的理解裡面是告訴編譯器可以往衍生的類別裡面去找有沒有 override 的版本

更進一步可以把函數定義成**純粹虛擬函數**（**pure virtual function**），有定義純粹虛擬函數的類別叫做抽象類別，這種類別不能被實例化（產生實際的物件的意思），一定要被繼承，繼承的類別裡面一定要 override 那些純粹虛擬函數。至於語法就是 `virtual` 搭配 `=0`，抽象類別大概長這樣

```cpp
class Container {
public:
    virtual double operator[](int) = 0;
    virtual int size() const = 0;
    virtual ~Container();
};
```

### `override`

這個關鍵字可寫可不寫，但寫了有很多好處：
1. 明確地表達你的意圖（不厭其煩再三強調）
2. 會幫你檢查拼字和型別之類的事情

```cpp
class Vector_container: public Container {
public:
    Vector_container(int s): v(s) {}
    ~Vector_container() {}

    double operator[](int i) override { return v[i]; }
    int size() const override { return v.size(); }
private:
    Vector v;
};
```

:::info
Notes：
1. 這邊新學到的東西應該只有 `override` 這個關鍵字，`virtual` 的部分算是複習
2. `override` 跟 Java 的 `@override` annotation 很像。Python 就比較沒有這種原生的功能，[Stack Overflow](https://stackoverflow.com/questions/1167617/in-python-how-do-i-indicate-im-overriding-a-method) 說是因為他沒有編譯期齊檢查，想想好像也是
:::

## 階層導航

如果想要知道「是哪個」衍生類別的話，用 `dynamic_cast`，例如以下的類別

```cpp
class Shape {
public:
    virtual ~Shape() = default;
};

class Circle: public Shape {};
class Triangle: public Shape {};
```

```cpp
Shape* s1 = new Circle();
Shape* s2 = new Triangle();

if (Circle* c = dynamic_cast<Circle*>(s1)) {
    std::cout << "This is a circle." << std::endl;
} else {
    std::cout << "This is not a circle." << std::endl;  
}

if (Circle* c = dynamic_cast<Circle*>(s2)) {
    std::cout << "This is a circle." << std::endl;
} else {
    std::cout << "This is not a circle." << std::endl;  
}
```

這是因為型別不符合時，`dynamic_cast` 會回傳 `nullptr`（忘記 `nullptr` 的看[這邊](https://hackmd.io/UkLiREP7Q_mKgI6Pun58YQ#Null-%E6%8C%87%E6%A8%99)）

也可以用抓例外的方式來做這件事（但我比較喜歡前一種方式）

```cpp
Shape* s1 {read_shape(cin)};
Circle& c {dynamic_cast<Circle&>(*s1)} // catch std::bad_cast
```

:::info
Notes：
1. 這個還蠻有趣的，雖然 C++ 裡面沒有像 Python 有 `type()` 這種方便的東西，但還是可以達成類似的事情
2. 做這個範例的時候有遇到一個錯誤 `error: 'Shape' is not polymorphic`，解決方法跟觀念解釋在[這裡](https://stackoverflow.com/questions/62408856/c-dynamic-cast-is-not-polymorphic)
:::

## 避免資源洩漏

使用類別的時候，常常會忘記把在 constructor 裡面 `new` 的東西 `delete` 掉，或是寫錯了 `delete` 超過一次，造成資源的洩漏或程式的不安全，這邊介紹了一個好用的東西 `unique_ptr`

如果不使用「聰明指標」，可能會寫出這樣的函式，一直在重複寫一樣的東西

```cpp
bool some_function() {
    Shape* s1 = new Shape();
    
    if (condition_1) {
        delete s1;
        return false;
    }
    
    if (condition_2) {
        delete s1;
        return false;
    }
    
    delete s1;
    return true;
}
```

使用了「聰明指標」，人生變美麗

```cpp
bool some_function() {
    std::unique_ptr<Shape> s1 = std::make_unique<Shape>();
    
    if (condition_1)
        return false;    
    if (condition_2)
        return false;
    return true;
}
```

:::info
Notes：之前寫資料結構的作業時確實有遇到資源洩漏的問題（雖然我都放著不理他幸好沒出事），不過系統越來越複雜時確實是需要注意，搞不好哪天不小心存取已經刪除的指標系統就掛了
> 使用 `unique_ptr` 或 `shared_ptr`（之後會介紹）來取代 `new` 跟 `delete`（4.5.3節）
:::

## Reference

### Operator Overloading
* Paul Deitel and Harvey Deitel, “C++ How to Program (late objects version)” Seventh Edition 
    * Chapter 11: Operator Overloading
* [為您的自訂類別多載 << 運算子](https://docs.microsoft.com/zh-tw/cpp/standard-library/overloading-the-output-operator-for-your-own-classes?view=vs-2019)，Microsoft
### Override
* [Java 什麼是覆寫(Override)](https://matthung0807.blogspot.com/2017/10/java-override.html)，菜鳥工程師 肉豬
* [In Python, how do I indicate I'm overriding a method?](https://stackoverflow.com/questions/1167617/in-python-how-do-i-indicate-im-overriding-a-method)，Stack Overflow
### Other
* [Bitwise Operators in C/C++](https://www.geeksforgeeks.org/bitwise-operators-in-c-cpp/)，GeeksforGeeks
* [initializer_list](http://www.cplusplus.com/reference/initializer_list/initializer_list/)，cplusplus.com - The C++ Resources Network
* [Determine the type of an object?](https://stackoverflow.com/questions/2225038/determine-the-type-of-an-object)，Stack Overflow
* [C++ Dynamic cast, is not polymorphic?](https://stackoverflow.com/questions/62408856/c-dynamic-cast-is-not-polymorphic)，Stack Overflow
* [DAY 3：只能死一次，不能鞭屍，談 std::unique_ptr<T>，卷一](https://ithelp.ithome.com.tw/articles/10213866)，山姆大叔談 C++：從歷史談起，再給個定義—Modern C++ 解惑，iT邦幫忙


