# Python Data Structures: Features & Examples

---

## 1. Keeps Insertion Order
**Does the data structure remember the order in which elements were added?**

- **List** ✅
  ```python
  lst = []
  lst.append('a')
  lst.append('b')
  lst.append('c')
  print(lst)  # ['a', 'b', 'c']
  print(lst[0])  # 'a'
  print(lst[1])  # 'b'
  ```
  Lists always remember the order items were added.

- **Set** ⚠️ (since Python 3.7, not guaranteed before)
  ```python
  s = set()
  s.add('a')
  s.add('b')
  s.add('c')
  print(s)  # {'a', 'b', 'c'}
  s2 = {'c', 'a', 'b'}
  print(s2)  # {'c', 'a', 'b'}
  ```
  Sets preserve insertion order in modern Python, but not guaranteed in older versions.

- **Dict** ✅ (since Python 3.7)
  ```python
  d = {}
  d['x'] = 1
  d['y'] = 2
  d['z'] = 3
  print(d)  # {'x': 1, 'y': 2, 'z': 3}
  for key in d:
      print(key, d[key])
  ```
  Dictionaries remember key insertion order.

- **Tuple** ✅
  ```python
  t = ('a', 'b', 'c')
  print(t)  # ('a', 'b', 'c')
  print(t[0])  # 'a'
  ```
  Tuples always keep insertion order.

---

## 2. Allows Duplicates
**Can the container store multiple copies of the same value?**

- **List** ✅
  ```python
  lst = ['apple', 'apple', 'banana']
  print(lst)  # ['apple', 'apple', 'banana']
  ```
  Lists allow duplicates.

- **Set** ❌
  ```python
  s = {'apple', 'apple', 'banana'}
  print(s)  # {'apple', 'banana'}
  s.add('apple')
  print(s)  # {'apple', 'banana'}
  ```
  Sets automatically remove duplicates.

- **Dict** (Keys: ❌, Values: ✅)
  ```python
  d = {'a': 1, 'b': 1, 'c': 2}
  print(d)  # {'a': 1, 'b': 1, 'c': 2}
  d['a'] = 99
  print(d)  # {'a': 99, 'b': 1, 'c': 2}
  ```
  Keys must be unique; values can duplicate.

- **Tuple** ✅
  ```python
  t = ('apple', 'apple', 'banana')
  print(t)  # ('apple', 'apple', 'banana')
  ```
  Tuples allow duplicates.

---

## 3. Mutable
**Can the data structure be changed after creation?**

- **List** ✅
  ```python
  lst = [1, 2, 3]
  lst.append(4)
  print(lst)  # [1, 2, 3, 4]
  lst[0] = 99
  print(lst)  # [99, 2, 3, 4]
  ```

- **Set** ✅
  ```python
  s = {1, 2, 3}
  s.add(4)
  print(s)  # {1, 2, 3, 4}
  s.remove(2)
  print(s)  # {1, 3, 4}
  ```

- **Dict** ✅
  ```python
  d = {'a': 1, 'b': 2}
  d['c'] = 3
  print(d)  # {'a': 1, 'b': 2, 'c': 3}
  d['a'] = 99
  print(d)  # {'a': 99, 'b': 2, 'c': 3}
  del d['b']
  print(d)  # {'a': 99, 'c': 3}
  ```

- **Tuple** ❌
  ```python
  t = (1, 2, 3)
  try:
      t[0] = 99
  except TypeError as e:
      print(e)  # 'tuple' object does not support item assignment
  ```
  Tuples are immutable.

---

## 4. Indexed Access
**Can you get an element by its position/index?**

- **List** ✅
  ```python
  lst = ['a', 'b', 'c']
  print(lst[1])  # 'b'
  ```

- **Set** ❌
  ```python
  s = {'a', 'b', 'c'}
  try:
      print(s[0])
  except TypeError as e:
      print(e)  # 'set' object is not subscriptable
  ```

- **Dict** ❌ (no numeric indexing)
  ```python
  d = {'a': 1, 'b': 2}
  try:
      print(d[0])
  except KeyError as e:
      print(e)  # 0
  print(list(d.keys())[0])  # 'a'
  print(d['a'])  # 1
  ```

- **Tuple** ✅
  ```python
  t = (10, 20, 30)
  print(t[2])  # 30
  ```

---

## 5. Sliceable
**Can you extract a subsequence like obj[start:end]?**

- **List** ✅
  ```python
  lst = [10, 20, 30, 40, 50]
  print(lst[1:4])  # [20, 30, 40]
  ```

- **Set** ❌
  ```python
  s = {10, 20, 30, 40, 50}
  try:
      print(s[1:3])
  except TypeError as e:
      print(e)  # 'set' object is not subscriptable
  # Convert to list to slice:
  print(list(s)[1:3])
  ```

- **Dict** ❌
  ```python
  d = {'a': 1, 'b': 2, 'c': 3}
  try:
      print(d[0:2])
  except TypeError as e:
      print(e)  # unhashable type: 'slice'
  # Slice keys as list:
  print(list(d.keys())[0:2])
  ```

- **Tuple** ✅
  ```python
  t = (10, 20, 30, 40, 50)
  print(t[1:4])  # (20, 30, 40)
  ```

---

## 6. Hashable
**Can the object be used as a key in a dict or element in a set?**

- **List** ❌
  ```python
  lst = [1, 2, 3]
  try:
      hash(lst)
  except TypeError as e:
      print(e)  # unhashable type: 'list'
  ```

- **Set** ❌
  ```python
  s = {1, 2, 3}
  try:
      hash(s)
  except TypeError as e:
      print(e)  # unhashable type: 'set'
  ```

- **Dict** ❌
  ```python
  d = {'a': 1}
  try:
      hash(d)
  except TypeError as e:
      print(e)  # unhashable type: 'dict'
  ```

- **Tuple** ✅ (if all elements hashable)
  ```python
  t1 = (1, 2, 3)
  print(hash(t1))  # Works fine
  t2 = ([1, 2], 3)
  try:
      hash(t2)
  except TypeError as e:
      print(e)  # unhashable type: 'list'
  ```

---

## 7. Key-Value Mapping
**Does the container associate keys to values?**

- **List** ❌
  ```python
  lst = ['a', 'b', 'c']
  # Only position/index, no keys
  ```

- **Set** ❌
  ```python
  s = {'a', 'b', 'c'}
  # Only unique elements, no keys
  ```

- **Dict** ✅
  ```python
  d = {'name': 'Alice', 'age': 30}
  print(d['name'])  # 'Alice'
  ```

- **Tuple** ❌
  ```python
  t = ('x', 'y', 'z')
  # Only values at positions, no keys
  ```

---

## 8. Can Be Dict Key
**Can you use this object itself as a dictionary key?**

- **List** ❌
  ```python
  lst = [1, 2]
  try:
      d = {lst: "value"}
  except TypeError as e:
      print(e)  # unhashable type: 'list'
  ```

- **Set** ❌
  ```python
  s = {1, 2}
  try:
      d = {s: "value"}
  except TypeError as e:
      print(e)  # unhashable type: 'set'
  ```

- **Dict** ❌
  ```python
  d1 = {'a': 1}
  try:
      d = {d1: "value"}
  except TypeError as e:
      print(e)  # unhashable type: 'dict'
  ```

- **Tuple** ✅ (if elements hashable)
  ```python
  t = (1, 2)
  d = {t: "value"}
  print(d[t])  # 'value'
  t2 = ([1, 2], 3)
  try:
      d = {t2: "value"}
  except TypeError as e:
      print(e)  # unhashable type: 'list'
  ```

