# [xieite](../xieite.md)::[functors](../functors.md)::ProcessGuard
Defined in header [<xieite/functors/ProcessGuard.hpp>](../../include/xieite/functors/ProcessGuard.hpp)

<br/>

Executes a callback when the program ends

<br/><br/>

## Synopsis

<br/>

```cpp
struct ProcessGuard {
	template<std::invocable<> Invocable>
	ProcessGuard(const Invocable&);

	void release();
};
```
### Public members
<pre><code>ProcessGuard/
|- <a href="./ProcessGuard/constructor.md">ProcessGuard</a>
`- <a href="./ProcessGuard/release.md">release</a>
</code></pre>

<br/><br/>

## Example
```cpp
#include <iostream>
#include <xieite/functors/ProcessGuard.hpp>

void foo() {
	xieite::functors::ProcessGuard guard([] {
		std::cout << "foo\n";
	});
}

int main() {
	foo();

	std::cout << "bar\n";
}
```
Output:
```
bar
foo
```
