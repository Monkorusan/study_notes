# Rigorous Proof of the Second Term in Partial Coverage Control

We want to rigorously show why

$$
\frac{\partial}{\partial p_i} \int_{E_i(p)} \varphi(q) \, dq = -\int_{\partial B(p_i, R) \cap S_i} \varphi(q) \, \frac{q - p_i}{\|q - p_i\|} \, dS(q)
$$

Here's the exact logic chain.

## 1️⃣ Define $E_i(p)$ properly

$$
E_i(p) = \{ q \in S_i(p) : \|q - p_i\| \geq R \}
$$

It's the portion of the Voronoi cell outside the sensing disk of radius $R$.

Its boundary has two pieces:

- the **fixed part** $\partial S_i(p)$ (depends on other agents but we'll ignore that for now), and
- the **moving part** $\partial B(p_i, R) \cap S_i$, which moves when $p_i$ moves.

Only this moving part contributes to $\frac{\partial}{\partial p_i}$, since the rest doesn't move w.r.t. $p_i$.

## 2️⃣ Write a small perturbation

Let the agent move:

$$
p_i \mapsto p_i + \varepsilon v_i, \quad \varepsilon \ll 1
$$

Then the ball boundary moves to

$$
\partial B(p_i + \varepsilon v_i, R) = \{ q : \|q - p_i - \varepsilon v_i\| = R \}
$$

## 3️⃣ Use Reynolds' transport theorem (a.k.a. shape derivative / Leibniz rule)

For any region $\Omega(\varepsilon)$ that moves smoothly with velocity field $u(q)$,

$$
\frac{d}{d\varepsilon}\bigg|_{\varepsilon=0} \int_{\Omega(\varepsilon)} f(q) \, dq = \int_{\partial \Omega(0)} f(q) \, u(q) \cdot n(q) \, dS(q)
$$

where $n(q)$ is the outward normal of $\Omega(0)$.

So the derivative of a moving-domain integral becomes a surface integral weighted by the normal velocity $u \cdot n$.

## 4️⃣ Compute the normal velocity on $\partial B(p_i, R)$

At a boundary point $q$, we have

$$
g(q, p_i) = \|q - p_i\| - R = 0
$$

Differentiate w.r.t. $\varepsilon$:

$$
\frac{d}{d\varepsilon} g(q(\varepsilon), p_i + \varepsilon v_i) = 0 \implies \nabla_q g \cdot \frac{dq}{d\varepsilon} + \frac{\partial g}{\partial p_i} \cdot v_i = 0
$$

But

$$
\nabla_q g = \frac{q - p_i}{\|q - p_i\|} = n(q), \quad \frac{\partial g}{\partial p_i} = -n(q)
$$

Hence

$$
n(q) \cdot \frac{dq}{d\varepsilon} - n(q) \cdot v_i = 0 \implies \frac{dq}{d\varepsilon} = v_i \implies u(q) = -v_i
$$

because from the fixed spatial frame, if the sphere's center moves by $+v_i$, the surface points move oppositely by $-v_i$.

Thus the normal velocity of the boundary relative to fixed coordinates is

$$
u \cdot n = -v_i \cdot n(q) = -v_i \cdot \frac{q - p_i}{\|q - p_i\|}
$$

## 5️⃣ Plug into Reynolds' formula

$$
\frac{d}{d\varepsilon}\bigg|_{\varepsilon=0} \int_{E_i(p_i + \varepsilon v_i)} \varphi(q) \, dq = \int_{\partial B(p_i, R) \cap S_i} \varphi(q) \, (-v_i \cdot n) \, dS
$$

Now, bring $v_i$ outside the integral (it's constant):

$$
\frac{d}{d\varepsilon}\bigg|_{\varepsilon=0} \int_{E_i(p_i + \varepsilon v_i)} \varphi(q) \, dq = -v_i \cdot \int_{\partial B(p_i, R) \cap S_i} \varphi(q) \, n(q) \, dS
$$

## 6️⃣ Identify the gradient (remove $v_i$)

By definition of directional derivative:

$$
\frac{\partial}{\partial p_i} \int_{E_i(p)} \varphi(q) \, dq = -\int_{\partial B(p_i, R) \cap S_i} \varphi(q) \, n(q) \, dS
$$

Substitute $n(q) = \frac{q - p_i}{\|q - p_i\|}$ to get:

$$
\frac{\partial}{\partial p_i} \int_{E_i(p)} \varphi(q) \, dq = -\int_{\partial B(p_i, R) \cap S_i} \varphi(q) \, \frac{q - p_i}{\|q - p_i\|} \, dS(q)
$$

---

## ✅ Summary

Now the missing pieces are clear:

- $E_i$ is a 3D (or 2D) region;
- $\partial B(p_i, R) \cap S_i$ is its moving boundary surface;
- $dS$ appears from the surface integral in Reynolds' theorem;
- $\delta p_i$ (or $v_i$) disappears when converting the directional derivative to the gradient vector.

---

# Bonus: About the $(b+R^2)$ Coefficient

Let's unpack why $b$ becomes $b+R^2$ in the rewritten form of the objective function.

## 1️⃣ Original objective in the second paper

$$
J(p,t) = -\sum_{i=1}^{n} \int_{S_i} \|q - p_i\|^2 \, \phi(q,t) \, dq + b \int_{Q \setminus \bigcup_i S_i} \phi(q,t) \, dq, \quad (b \leq -R^2)
$$

Here:

- $S_i = B(p_i, R) \cap V_i$: the visible (or sensed) region of agent $i$,
- $Q$: the mission domain,
- $\phi(q,t)$: density (importance) function,
- $b$: penalty constant for uncovered areas.

## 2️⃣ Rewriting step and the appearance of $(b+R^2)$

When the authors re-express the objective to unify terms, they often use the fact that

$$
Q = \bigcup_i S_i \cup \left(Q \setminus \bigcup_i S_i\right),
$$

so the second integral can be rewritten as an integral over $Q$ minus the sum over $S_i$.

After algebraic manipulation:

$$
J(p,t) = b \int_Q \phi(q,t) \, dq - \sum_i \int_{S_i} \left(\|q - p_i\|^2 + b\right) \phi(q,t) \, dq.
$$

Now, if $b \leq -R^2$, one can define

$$
b' = b + R^2,
$$

so that

$$
\|q - p_i\|^2 + b = \left(\|q - p_i\|^2 - R^2\right) + (b + R^2) = \left(\|q - p_i\|^2 - R^2\right) + b'.
$$

That substitution isolates the term $(\|q - p_i\|^2 - R^2)$, which is zero at the boundary of the sensing disk (where $\|q - p_i\| = R$).

This is conceptually convenient — it makes the coverage potential reference to the sensing limit rather than absolute distance.

## 3️⃣ Physical meaning

- $b$: represents how undesirable an unsensed (uncovered) point is.
- $R^2$: represents the maximum sensing cost — the squared radius of sensing region.

So $b + R^2$ acts as an effective penalty offset, ensuring continuity of the objective at the boundary of each sensor's coverage disk.

In other words, by adding $R^2$, the cost function is normalized so that the integrand inside $S_i$ transitions smoothly to zero at the edge of the sensing region, avoiding discontinuities when regions expand or shrink.

## ✅ Summary

They replace $b$ with $b + R^2$ not arbitrarily, but to shift the reference so that the potential inside each sensor's domain is zero at the sensing boundary ($\|q - p_i\| = R$). This keeps the objective physically meaningful and differentiable when defining gradient-based coverage controllers.
