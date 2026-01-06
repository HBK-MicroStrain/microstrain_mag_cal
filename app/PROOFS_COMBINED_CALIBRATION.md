# Combined Calibration Proofs
This document contains formal proofs for the derived equations that calculate the combined 
calibration coefficients. 

These coefficients are necessary since the data sampled for calibration
was likely already calibrated, unless the calibration was zeroed out first. The combined coefficients
account for this and allow the new calibration to be applied on top of the old one.

## Combined Soft-Iron Correction Matrix
**Goal:** Find $\mathbf{A}_{\text{combined}}$ such that:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}} - \mathbf{b}_{\text{combined}})
$$

**Given:** The correct equation:

$$
\mathbf{m}_{\text{corr}} = \mathbf{A}(\mathbf{m}_{\text{raw}} - \mathbf{b})
$$

**Derivation:**

Starting with the composition of transformations:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{m}_{\text{old}} - \mathbf{b}_{\text{new}})
$$

**Step 1:** Substitute the expression for $\mathbf{m}_{\text{old}}$:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}} - \mathbf{b}_{\text{old}}) - \mathbf{b}_{\text{new}})
$$

**Step 2:** Apply the distributive property on $\mathbf{A}_{\text{old}}$:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}}) - \mathbf{b}_{\text{new}})
$$

**Step 3:** Apply the distributive property on $\mathbf{A}_{\text{new}}$:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) - \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})
$$

**Step 4:** Factor out the constant offset terms with the distributive property:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))
$$

**Step 5:** Compare with the desired form:

*Desired*: $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}} - \mathbf{b}_{\text{combined}})$

*Derived:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))$

**Step 6:** Applying the distributive property to the desired form:

*Desired*: $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}})$

*Derived:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))$

**Step 7:** Substitute the *desired* expression for $\mathbf{m_new}$:

$$
\mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})),
$$

for all possible values of $\mathbf{m}_{\text{raw}}$.

**Step 8:** Rearrange to group terms:

$$
(\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}})
$$

**Step 9:** Factor the right side using the distributive property:

$$
(\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}) - \mathbf{A}_{\text{combined}})\mathbf{m}_{\text{raw}}
$$

**Step 10:** Substitute zero for the left side:

For the equation to hold for all values of $\mathbf{m}_{\text{raw}}$, both sides must be 0. Therefore:

$$
0 = (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}) - \mathbf{A}_{\text{combined}})\mathbf{m}_{\text{raw}}, \text{ for all } \mathbf{m}_{\text{raw}}
$$

**Step 11:** For this to be true for all $\mathbf{m}_{\text{raw}}$, we require:

$$
0 = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}) - \mathbf{A}_{\text{combined}}
$$

**Step 12:** Rearrange for the derived equation:

$$
\mathbf{A}_{\text{combined}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})
$$

## Combined Hard-Iron Offset Vector

**Derivation:**

Starting from *step 4* above, we have:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))
$$

and we have established that:

$$
\mathbf{A}_{\text{combined}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})
$$

**Step 1:** Write the explicit desired form:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}})
$$

**Step 2:** Substitute $\mathbf{A}_{\text{combined}}$:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{combined}})
$$

**Step 3:** Match the constant terms from both expressions

*Desired:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{combined}})$

*Derived:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{m}_{\text{raw}}) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))$

**Step 4:** The constant terms (without $\mathbf{m}_{\text{raw}}$) must be equal:

$$
\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{combined}}) = (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}})(\mathbf{b}_{\text{old}}) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))
$$

**Step 5:** Left-multiply both sides by $\mathbf{A}_{\text{new}}^{-1}$:

$$
\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{combined}}) = \mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}}) + \mathbf{b}_{\text{new}}
$$

**Step 6:** Left-multiply both sides by $\mathbf{A}_{\text{old}}^{-1}$ for the derived equation:

$$
\mathbf{b}_{\text{combined}} = \mathbf{b}_{\text{old}} + \mathbf{A}_{\text{old}}^{-1}(\mathbf{b}_{\text{new}})
$$
