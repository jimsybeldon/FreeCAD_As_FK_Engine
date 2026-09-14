# FreeCAD Python 3.11 & PyCharm Setup Guide

This reference guide summarizes the steps required to link a standalone Python 3.11 environment to FreeCAD 1.1's C++ binaries, enabling script execution and IDE support inside PyCharm without GUI dependency.

---


```markdown



## 1. PyCharm IDE Configuration

To enable `import FreeCAD` and `import Part` without encountering unresolved import warnings or missing path errors:

1. Open **PyCharm Settings** (`Ctrl + Alt + S` on Windows/Linux, `Cmd + ,` on macOS).
2. Navigate to **Project: <your_project_name>** $\rightarrow$ **Project Structure**.
3. In the right panel, click **+ Add Content Root**.
4. Select your FreeCAD 1.1 binary installation path:
   * **Windows:** `C:\Program Files\FreeCAD 1.1\bin`
   * **Linux:** `/usr/lib/freecad/lib` (or `/usr/lib/freecad-python3/lib`)
   * **macOS:** `/Applications/FreeCAD.app/Contents/Resources/lib`
5. Click **Apply** and **OK**.

### Optional: Autocompletion & Type Stubs
To enable code completion and type hinting for compiled C++ objects in PyCharm, install the stub definitions into your project's virtual environment:

```bash
pip install freecad-stubs

```

---

## 2. In-Script Environment Initialization

Because FreeCAD modules (`FreeCAD.pyd`, `Part.pyd`, etc.) are compiled C++ shared libraries rather than standard PyPI packages, your entry point script must dynamically append the installation directory to Python's system path and DLL search paths before invoking imports.

Add the following block to the very top of your Python execution script:

```python
import sys
import os

# Define absolute path to the FreeCAD 1.1 binary directory
freecad_bin_path = r"C:\Program Files\FreeCAD 1.1\bin"  # Adjust for your operating system

# Append FreeCAD libraries to Python's sys.path if not already present
if freecad_bin_path not in sys.path:
    sys.path.append(freecad_bin_path)

# On Windows (Python 3.8+), register the DLL directory to resolve C++ shared dependencies
if os.name == "nt" and hasattr(os, "add_dll_directory"):
    try:
        os.add_dll_directory(freecad_bin_path)
    except Exception as e:
        print(f"Warning: Failed to add DLL directory: {e}")

# Load FreeCAD C++ core modules
import FreeCAD as App
import Part

```

---

## 3. Headless Document Handling Best Practice

When executing scripts outside the FreeCAD GUI (headless via Python), avoid using `App.getDocument("doc_name")` directly to check for document existence, as it throws a C++ exception if the document is not found.

Use `App.listDocuments()` instead:

```python
def initialize_document(doc_name="FK_Engine_POC"):
    # Safely close existing document if re-running script in same process
    if doc_name in App.listDocuments():
        App.closeDocument(doc_name)
    
    # Create fresh document instance
    doc = App.newDocument(doc_name)
    return doc

```

```

```