# gcufl::Matrix<V>::Matrix
```cpp
Matrix(const std::vector<std::size_t>& dimensions) noexcept;

Matrix(const std::vector<std::size_t>& dimensions, const std::vector<V>& values);
```
Creates a multidimensional matrix.
## Example
```cpp
#include <gcufl/Matrix.hpp>

int main() {
	gcufl::Matrix<bool> matrix1({ 3, 3, 3 });

	gcufl::Matrix<int> matrix2({ 2, 2 }, { 1, 2, 3, 4 });
}
```