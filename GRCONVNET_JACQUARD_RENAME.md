# GR-ConvNet Model File Rename: Cornell → Jacquard

## ✅ Change Summary

**Old filename**: `grconvnet_cornell.pt`  
**New filename**: `grconvnet_jacquard.pt`

**Reason**: The model was trained on the **Jacquard Grasping Dataset**, not Cornell, so the filename should reflect the actual training data source for clarity.

---

## 📁 File Location

```
src/resources/ml_models/grconvnet_weights/grconvnet_jacquard.pt
```

---

## 🔄 Updated References

All references have been updated in:

1. ✅ **`src/config/config.py`** (line 212)

   - `GRCONVNET_MODEL_PATH` updated
   - Comments clarified: "Model trained on Jacquard Grasping Dataset"

2. ✅ **`debug_grconvnet_preprocessing.py`** (line 97)

   - `weights_path` updated

3. ✅ **`visualize_grconvnet_temporal.py`** (line 173)

   - `torch.load()` path updated

4. ✅ **`visualize_grconvnet_outputs.py`** (line 73)

   - `torch.load()` path updated

5. ✅ **`GR_ConvNet_Migration_Plan_FINAL.md`** (lines 214, 422)

   - Documentation updated

6. ✅ **`GR_ConvNet_Migration_Plan.md`** (line 691)
   - Older migration plan updated

---

## 📊 Dataset Comparison

| Dataset      | Size           | Objects         | Annotations             | Image Type |
| ------------ | -------------- | --------------- | ----------------------- | ---------- |
| **Cornell**  | ~1,000 images  | 240 objects     | Parallel-jaw rectangles | RGB-D      |
| **Jacquard** | ~54,000 images | 11,000+ objects | Parallel-jaw rectangles | RGB-D      |

**Jacquard is ~50x larger** and has better diversity, which is why GR-ConvNet uses it for training.

---

## 🎯 Implications

### Better Performance Expected:

- ✅ More object diversity (11k vs 240 objects)
- ✅ Better generalization to novel objects
- ✅ More robust to lighting/texture variations
- ✅ Improved diagonal angle predictions (hopefully!)

### Key Differences from Cornell:

- **Jacquard**: Industrial parts, tools, household items
- **Cornell**: Mostly household items (smaller variety)
- **Jacquard**: Multiple viewing angles captured
- **Cornell**: Single top-down view

---

## ⚠️ Note on Diagonal Grasps

The diagonal grasp issue (top-left to bottom-right objects) may be related to:

- **Dataset bias**: Both Cornell and Jacquard may have axis-aligned bias
- **Annotation style**: Parallel-jaw rectangles favor canonical orientations
- **Training augmentation**: Random rotations during training may not be sufficient

Even with Jacquard's larger size, diagonal grasps may still be underrepresented if:

1. Most objects are photographed in canonical orientations
2. Data augmentation doesn't emphasize diagonal angles
3. Loss function doesn't weight diagonal angles equally

---

## 🧪 Next Steps

1. **Test with diagonal objects** to see if Jacquard training helps
2. **Log angle distribution** across multiple test runs
3. **Compare with GGCNN2** (trained on Cornell) for diagonal performance
4. **Consider fine-tuning** on custom diagonal-heavy dataset if issue persists

---

## ✅ Status

- [x] File renamed: `grconvnet_cornell.pt` → `grconvnet_jacquard.pt`
- [x] All code references updated (7 files)
- [x] Documentation updated
- [x] Comments clarified ("Jacquard Grasping Dataset")
- [x] File verified to exist

**The rename is complete and all systems updated!** 🎉
