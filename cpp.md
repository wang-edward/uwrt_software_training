# Introduction to Modern C++

If the reader of this guide is currently in engineering and hasn't done C++ outside of school, there are some parts that are important to learn and understand before any code is written for training. The UWRT software team uses C++11 and usually C++ learnt in school is C++98. In this guide, there will be some review on important ideas such as memory and OOP while also introducing new concepts important in modern C++ that the software team uses. 

This guide is written with the assumption that the reader already understands the basics of C++. Ideas such as variables, conditional statements and loops should be understood before reading any further. This means that this guide is written to be educational and tries not only to introduce new programming concepts but also explain why they are necessary. Please try to read the guide in order, the concepts build on top of each other.

There will be included code snippets as examples incase there is any confusion. Feel free to refer to this guide whenever there is confusion on code.

Questions and Feedback can be directed to andrewzhao_, eady or mollaATime on discord. (Feel free to ask questions and voice any complaints or suggestions for improving this guide.)


**List of Concepts in this Guide**

- Vectors

- `auto` type

- Lambda functions

- Iterable for loops

- Memory 

- Object Oriented Programming

- Smart Pointers

- Lvalues, Rvalues

- Copy and Move Semantics

- Templates


#Vectors

Before talking about a vector, it's important to understand why a Vector is needed. This is understood by looking at the limitations of an array. The idea of an array is it allows for data that is grouped together to be represented by one variable. For example, a new array is created below:

`int array[] = {2, 4, 10, 5, 15, 3};`

![Array Image](Images/array.png)

Hopefully both the code and the image make intuitive sense as  this guide won't spend too much time explaining arrays. 

There are some major limitations with an array.

1) **Fixed Capacity**
    If `array` were to double in size, there would need to be a new array, `new_array[]`, created with all the values in `array` copied to `new_array[]`. This is tedious to program and test.
2) **Out of Bounds Elements**
    Trying to access elements outside of the array causes memory accesses errors that crash the program.

Thankfully, C++11 solves the aforementioned problems with `Vectors`.

Vectors can be thought of as dynamically sized arrays with functions to help with memory protection. Vectors are created a bit differently than arrays are in C++. Vectors are read and modified with square brackets [] similar to arrays, however there are also functions that add additional protection that are preferred instead. 


**Vector Creation**

`std::vector<int> vector = {2, 4, 10, 5, 15, 3}`

This is the syntax to create a vector with the name `vector` that has the same elements as the array up above.

`vector` can be modified and read using square brackets as well. The angle brackets might seem foreign and will be explained in the next section on Templates. (Bonus points if their beheaviour can be guessed)

If code isn't compiling, make sure the vector header is included in the source code before compiling.


**Adding and Removing New Elements** 

So creation of a vector is slightly different than an array but the behaviour seems to be pretty similar so far. However this is something that is exclusive to a vector, dynamic sizing. One of the biggest issues that were mentioned with arrays were that they had difficulty changing size. Once an array of size 6 is declared, it will always have 6 elements and can never change. 

For a vector, adding new elements using `push_back()` handles the complexities of resizing. 

Given the above example if I wanted to add a 7th element to `vector`, all I have to do is call `vector.push_back(5)` and now my vector has a 7th element that is appended to the end of the vector. 

If I want to delete an element at the end of the vector then I call `vector.pop_back()`.


**Memory Protection**

Earlier, it was mentioned that elements in the vector can be read and modified with square brackets similar to an array. However, this may lead to unexpected behaviour if indexing out of bounds. 

Vectors solve this issue with `at()` function call. Reading a value is done with `vector.at(index)` and setting a value is done by assigning a value `vector.at(index)`. There will be a bounds check done by `at()` before assignment. 


In summary, this table illustrates the parallels and differences between vectors and arrays.

||Array|Vector|
|-|-----|------| 
|Creating| `int array[] = {};` | `std::vector<int> vector = {};`|
|Reading| `array[index];` | `vector.at(index);`|
|Modifying|`array[index] = new_value;` | `vector.at(index) = new_value;`|
|Adding New Element| NA | `vector.push_back(new_value);`|
|Removing New Element| NA | `vector.pop_back();`|




#auto Keyword

C++ is known for requiring a type for every variable. This remains true in C++11 but there are times when the programmer doesn't care about the exact type . The `auto` keyword allows for creating a variable with the type being determined at runtime instead of compile time. However, `auto` should be used sparingly and not as a replacement for types as it leads to poor coding practices and undefined behaviour. 

For example

`auto counter = {1, 2, 4, 5};`

Is difficult to debug and use since counter's type is determined at runtime and if counter is used in the future, there can be unexpected behaviour. 


`auto` is better off used whenever the type itself is very obvious to the programmer such as creating lambda functions or iterable for loops.


#Lambda Functions

The advantage of functions in programming instead of using just the main function is the ability to isolate code, create better readability and allow for more reusable code. However, sometimes, portions of logic are only used once and the additional overhead of writing a function just isn't worth the tradeoff. C++11 tries to find a balance by introducing Lambda functions. Lambdas are inline functions that are defined inside the code itself. The syntax is as follows:


`auto function = [capture clauses](parameter list) -> return type { function definition }`

This is one of the instances where using `auto` is ok as the function itself is just a pointer and the type of the pointer is explicitly defined in the lambda. Requiring an explicit declaration would be superfluous.

`int main() {`
  
    int x = 5;
    auto function = [](int x) -> int {return x += 2;};
    std::cout << function(x) << std::endl;
`}`

It is expected that x is 7.

Most of the lambda is self explanatory except for the capture clauses. Capture clauses allow for a lambda to access variables that are of the parent scope. There won't be too much depth for the capture clauses explanation here and for most cases leaving the field blank will suffice. For a through explanation, check out [Microsoft's Lambda Function Documentation](https://learn.microsoft.com/en-us/cpp/cpp/lambda-expressions-in-cpp?view=msvc-170)


Although this is a crude example, proper use of lambdas allows for better code isolation and readability without much extra overhead. 


#Iterable For Loops

This is another instance where `auto` is permitted. Typically, for loops in C++ are defined as:

![For Loops](Images/c-for-statement.png)


These types of loops are still commonly used in C++11 but an alternative called the iterable for loop is sometimes preferred or necessary. 

The iterable loop is defined as 

`for (auto iterator : iterable_list<type T>) {`
`}`

Once again the angled brackets will be explained later. Upon first inspection, this seems much less straight forward than the standard for loop and in some cases, it is also less useful than a standard for loop.

However consider an example of looping through a vector to get the size of all elements inside. Remember that vectors are dynamically sized.

`std::vector<int> list = {1, 2, 3}`

Earlier, it was explained that this vector can be appended to and deleted from while the code is running. Using a standard for loop, there are two approaches to this problem:
1) Using a variable to track the number of elements to iterate through.
2) Using the built in `size()` member function of the vector.

The second approach does seem more appealing. However, look at the code

`for (int i = 0; i < list.size(); i++) {`
`   return_sum += list.at(i);`
`}`

It isn't very visually appealing. Perhaps you're not convinced. So lets make list a 3x3 matrix and try summing that instead

`for (int i = 0; i < list.at(0).size(); i++){`
`   for(int j = 0; j < list.size(); j++) {`
`       return_sum += list.at(i).at(j);`
`   }`
`}`


Let's compare that to how the iterable for loop would preform the summation

`for(auto element : list) {`
`   return_sum += element;`
`}`

And for the matrix example

`for (auto row : list){`
`   for(auto column : row) {`
`       return_sum += column;`
`   }`
`}`


Its easier to see what is going on in the code! 

The reason `auto` is permitted is that `list` is defined to be of integer type and so at run time, its obvious what type the iterators will be will be as well. If there is no `size()` member function for a data structure, then an iterable for loop is required if there is an iterable type. 

The standard for loop does have some advantages as earlier mentioned. For one, the iterable for loop makes it more difficult and unreadable if finding a specific index of the vector is important.


#Memory

This next section is a bit of review. Memory and memory allocation are notorious for being difficult to understand in C++. This section is very important to make sure that the reader is clear in their understanding before continuing any further.


First off, what is memory? Why do we need it?

If a computer didn't have any memory (or registers if you want to nitpick) there would be no way to save data as variables, call functions or even run any code!

Memory is extremely important and our primary concern is with variables and data. 

Without even knowing it, most variables created already allocate and use memory. Consider the following line of code: 

`int x = 5;`

This line of code creates a variable called `x` that uses memory in order to store the value 5 for further use. Before continuing any further, it is important to distinguish between the two major pieces of memory that a C++ programmer interacts with:

1) Stack
2) Heap


**Stack**
ront of me 
Without using keywords such as `new` or function calls such as `malloc()`, data is being stored on the stack. Variable declarations such as `int x = 5;`, `float y = 2.0f;` or `char z = 'c';` are all being stored onto the stack. 

So why not just always use the stack? Why was the heap mentioned?

The stack can only allocate memory for fixed size objects at compile time. Simply put, if I want an array where the size of the array is determined while my code is running, I cannot use the stack.


**Heap**

If there is data where the amount of memory the data takes up is only known while running, the heap has to be used.

The heap is memory that is protected (by the OS) and must be requested in order to use. The amount requested depends on the programmer, allowing for much more flexibility than the stack. 

To use memory from the heap, C++ has the `new` keyword to request memory.

`int * x = new int(5);`

There is now a variable x that has the value 5 on the heap. However, the syntax of using heap memory is much different than the stack memory. There is a `*` following `int` called a pointer and so x is no longer an integer it is now a pointer to an integer. 


This is confusing upon first learning however, the following sections illustrate what is going on in a detailed manner.


**Pointers** 

It was mentioned that when using the heap, x is a pointer to an integer rather than an integer like using the stack.

To see how this is actually different, try printing x on both the stack and the heap:

`int x = 5;`
`std::cout << x << std::endl;`

This results in 5 being printed out which is what is expected. 

However try printing out the pointer,

`int * x = new int(5);`
`std::cout << x << std::endl;`


This will almost never print 5, in fact it will usually result in some large random number or there might be a 0x followed by some random numbers. What is even more strange is `x` will never print the same value in a row when rerunning. 

A pointer, like the example above illustrates, is not the same as an integer. A pointer points to a memory address, an address that says exactly where my piece of memory is in my computer, hence the name pointer. The values that were being printed when x was on the heap were memory addresses where x was located. Before explaining what a memory address is, to verify that x being 5 actually does exist on memory, try running:

`int * x = new int(5);`
`std::cout << *x << std::endl;`

Miraculously, 5 is printed! The value at the memory address was printed and not the address itself. 

If this is a bit confusing, picture this analogy. 

Imagine I had two sticky labels with the number 5 on them and put one on a wall in my house and another on my friend who leaves my house after I give him the label but I can always ask for his location if I want to see the label again. 

Let's say a year passed and I remember I had the labels on the wall and on my friend but I had forgotten what was on the label and I really wanted to know. 

If I wanted to know the value of the label on the wall, I already know where the wall is so I can go directly to where the wall is and read the value. The wall doesn't move so it's easy to get the value on the label.

However, if I wanted to see the value of the label on my friend, the only way I can do that is if I have his location or address first. This location may change depending on where he lives. 

Hopefully, this paints an idea of what a memory address is before formally explaining it in the next section.


**Memory Addresses**


In order to find where a section of memory is in the computer, an address to that memory is required. Regardless if my data lies on my stack or my heap, there is always an address. In the previous section, the analogy for finding the value on a label between the wall and friend still requires an address for the wall. The address isn't needed in order to find the value on the label but it still exists if required, it would be your own home address. 

Likewise, regardless if data is stored on the stack or the heap, there is always an address to that data. 

To obtain the address of a variable, prefix a variable with an ampersand `&`.

For example, `&x` will return a hex number the represents the memory address of where x is. 

To obtain the value of data at a memory address, prefix a variable with a star `*`.

For example, if `x` is an address/pointer, then `*x` will return the value at the address `x`.

For a pointer to an integer, the pointer is the address where the integer resides at. However, what happens when taking the address of a pointer? Try this example:

`int * x = new int(5);`
`std::cout << x << std::endl;`
`std::cout << &x << std::endl;`

Intrestingly enough, a different address then the pointer is printed out, and suprisingly, this address is actually the stack (seem confusing?).


Hopefully this example can illustrate what's happening in code:

If a pointer with the value 5 is created.

`int * x = new int(5);`
`std::cout << x << std::endl;`

If `x` prints 0x008 and `*x` prints 5 then this is a visual representation of heap memory at the moment.

Heap Memory 
|Addresses|Values|
|-|-|
|0x008 | 5 |

At memory location 0x008, 5 is stored.

In order to print `x`, the value 0x008 or the pointer to the heap memory also has to be stored in memory as well. However, this pointer is not stored in the heap but the stack.

When this line of code is run

`std::cout << &x << std::endl;`

if 0x004 is returned, this can be visualized as following in the stack memory.

Stack Memory


|Addresses|Values| 
|-|-| 
|0x004 | 0x008 |

At memory location 0x004, 0x008 is stored.

So `x` the pointer is the address of the heap data which contains 5 but it's not the same as the address of `x` the pointer.

Following the analogy from the previous section, to find the address of my friend, I have to use my phone to find him. However, my phone also has an address itself which isn't the location of my friend but the address of the device that helps find my friend.

**Freeing Memory**

This is the most important section to understand the next parts. 

After acquiring heap data, the developer is allowed to do anything with it. However, before exiting a program, it is paramount that the data requested from the heap is returned back to it when done. This is done using the `delete` or `delete[]` calls.

Failure to do so causes memory faults that are the bane of C++ programming. In 2019, it was reported by Microsoft that 70% of patches for Windows in the past 12 years were to fix memory fault bugs caused by C and C++. 

Good code etiquette requires using delete and delete[] as many times as new has been called to minimize these memory issues. In fact, some companies like NASA and Google take it to the extreme by strictly preventing developers from requesting memory from the heap after initialization and sometimes even at all.


So should we not use the heap at all?

Well not really, it still is very powerful whenever dynamic memory allocation is needed. However, compared to other programming languages like Java, C++ doesn't have garbage collection built into the language to gurantee that data is cleaned up after it's been done being used. Fortunately, C++11 introduces new tools that rely on object oriented programming to offer a solution. Before introducing them, a review of OOP is necessary.


#Object Oriented Programming


There are four main pillars of object oriented programming 

- Abstraction
- Encapsulation
- Polymorphism
- Inheritance

Before giving some explanations of them, it helps to first understand what actually is an object.

**Objects**

Objects in C++ are a way to group data together that makes logical sense. Combined with the four pillars mentioned up above, expressions that can't be done with procedural programming (Logic going line by line only using simple variables and functions) feels natural using objects. 

Imagine trying to create a program that tracks various attributes of a person. This example will be used for the entire OOP section. The height, weight, hair color, eye color and sex are all required to be tracked.

If this is written procedurally, 











In layman's terms what does that mean?


**Abstraction**


    


#Templates

Earlier, it was mentioned that the angle brackets used to create a vector would be further explained in this section. Templates allow for a function or data structure that uses various different types to rely on the same definition. 

What does this actually mean?

In C++98, if 