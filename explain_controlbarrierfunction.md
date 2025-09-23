# Line-by-Line Explanation: ControlBarrierFunction(ABC)

```python
class ControlBarrierFunction(ABC):
    """Abstract class of control barrier function."""
    get_positions: Callable[[], np.ndarray]
    enable: bool = True
    grad_num: int

    @abstractmethod
    def _calc_cbf(self, positions: np.ndarray):
        """Calculate and return CBF."""

    @abstractmethod
    def _calc_grad(self, positions: np.ndarray):
        """Calculate and return gradient of CBF."""

    @property
    def cbf(self):
        """Get CBF value by revoking _calc_cbf method."""
        if self.enable:
            return self._calc_cbf(self.get_positions())
        else:
            return np.zeros((1))

    @property
    def grad(self):
        """Get gradient array by revoking _calc_grad method."""
        if self.enable:
            return self._calc_grad(self.get_positions())
        else:
            return np.zeros((self.grad_num))
```

## Explanation

- `class ControlBarrierFunction(ABC):`
  - Defines a new class called `ControlBarrierFunction` that inherits from `ABC` (Abstract Base Class). This means it's a template for other classes and can't be used directly.

- `"""Abstract class of control barrier function."""`
  - This is a docstring. It describes what the class is for. Here, it says this is an abstract class for control barrier functions.

- `get_positions: Callable[[], np.ndarray]`
  - This is a type hint. It says that `get_positions` should be a function that takes no arguments and returns a numpy array. It's not assigned here, just declared for clarity.

- `enable: bool = True`
  - This creates a class variable called `enable` and sets it to `True` by default. It is used to turn the function on or off.

- `grad_num: int`
  - Another type hint. It says `grad_num` should be an integer. It is not assigned a value here.

- `@abstractmethod`
  - This is a decorator. It marks the following method as abstract, meaning subclasses must implement it.

- `def _calc_cbf(self, positions: np.ndarray):`
  - This defines an abstract method called `_calc_cbf`. It takes `positions` (a numpy array) as input. No logic is provided here, just the method signature.

- `def _calc_grad(self, positions: np.ndarray):`
  - Another abstract method. Subclasses must provide the logic for this method.

- `@property`
  - This decorator makes the following method act like an attribute. You can access it with `obj.cbf` instead of `obj.cbf()`.

- `def cbf(self):`
  - This defines a property called `cbf`. It checks if `enable` is `True`. If so, it calls `_calc_cbf` using the current positions. Otherwise, it returns a numpy array of zeros.

- `def grad(self):`
  - Similar to `cbf`, this defines a property called `grad`. It calls `_calc_grad` if enabled, otherwise returns zeros.

## Summary
This class is a template for control barrier functions. It requires subclasses to implement two methods (`_calc_cbf` and `_calc_grad`). It also provides two properties (`cbf` and `grad`) that use those methods to get results, but only if enabled. The type hints help clarify what kind of data is expected.
