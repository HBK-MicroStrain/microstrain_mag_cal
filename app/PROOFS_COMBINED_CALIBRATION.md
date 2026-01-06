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
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) - \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})
$$

**Step 4:** Factor out the constant offset terms with the distributive property:

$$
\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))
$$

**Step 5:** Compare with the desired form:

*Desired*: $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}} - \mathbf{b}_{\text{combined}})$

*Derived:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))$

**Step 6:** Applying the distributive property to the desired form:

*Desired*: $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}})$

*Derived:* $\mathbf{m}_{\text{new}} = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}}))$

**Step 7:** Substitute the *desired* expression for $\mathbf{m_new}$:

$$
\mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}}) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})),
$$

for all possible values of $\mathbf{m}_{\text{raw}}$.

**Step 8:** Rearrange to group terms:

$$
(\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = \mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{m}_{\text{raw}})) - \mathbf{A}_{\text{combined}}(\mathbf{m}_{\text{raw}})
$$

**Step 9:** Factor the right side using the distributive property:

$$
(\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}(\mathbf{b}_{\text{old}})) + \mathbf{A}_{\text{new}}(\mathbf{b}_{\text{new}})) - \mathbf{A}_{\text{combined}}(\mathbf{b}_{\text{combined}}) = (\mathbf{A}_{\text{new}}(\mathbf{A}_{\text{old}}) - \mathbf{A}_{\text{combined}})\mathbf{m}_{\text{raw}}
$$
