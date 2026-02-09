# Testing Patterns

**Analysis Date:** 2026-02-09

## Test Framework

**Runner:**
- C++: Google Test (GTest) with GMock
  - Config: `CMakeLists.txt` in `myactuator_rmd/` defines GTest setup
  - Executable: `run_tests` (compiled from `test/run_tests.cpp`)

**Python:**
- pytest framework (ROS 2 standard)
- Linting tests via ament_flake8 and ament_pep257
- No standalone Python unit tests found for driver code; integration tests via ROS 2

**Assertion Library:**
- C++: Google Test assertions (`EXPECT_EQ`, `EXPECT_CALL`, `WillOnce`)
- Python: pytest assertions and ament test macros

**Run Commands:**
```bash
# C++ tests (myactuator_rmd)
cd myactuator_rmd/build
cmake .. -DBUILD_TESTING=on
make -j$(nproc)
ctest                          # Run all tests
ctest --verbose                # Verbose output
ctest -R <test_name>          # Run specific test

# With coverage
cmake .. -DBUILD_TESTING=on -DENABLE_COVERAGE=on
make -j$(nproc)
ctest
gcovr -j $(nproc) --filter include --filter src --print-summary --xml coverage.xml

# Python/ROS tests (minimal - mostly linting)
colcon test --packages-select TandP_URDF_description
```

## Test File Organization

**Location:**
- C++ tests: `myactuator_rmd/test/` directory with subdirectories for categories
- Python tests: `TandP_URDF_description/test/` for URDF package linting tests
- No unit tests found for Python driver modules (`myactuator_python_driver/`)

**Naming:**
- C++: `*_test.cpp` and `*_test.hpp` (e.g., `actuator_test.cpp`, `requests_test.cpp`)
- Python: `test_*.py` (e.g., `test_flake8.py`, `test_pep257.py`)

**Structure:**
```
myactuator_rmd/test/
├── actuator_test.cpp           # ActuatorInterface tests
├── can/
│   └── utilities_test.cpp       # CAN utility tests
├── protocol/
│   ├── requests_test.cpp        # Request message parsing
│   └── responses_test.cpp       # Response message parsing
├── mock/
│   ├── actuator_mock.cpp        # Mock actuator implementation
│   ├── actuator_mock.hpp
│   ├── actuator_actuator_mock_test.hpp  # Test fixture
│   └── actuator_actuator_mock_test.cpp
└── run_tests.cpp               # Test runner

TandP_URDF_description/test/
├── test_copyright.py           # License header checks
├── test_flake8.py              # Code style linting
└── test_pep257.py              # Docstring format checks
```

## Test Structure

**Suite Organization (C++):**
```cpp
namespace myactuator_rmd {
  namespace test {

    TEST(TestSuiteName, test_name) {
      // Test body
      EXPECT_EQ(actual, expected);
    }

    TEST_F(TestFixtureName, test_name) {
      // Uses fixture setup/teardown
      EXPECT_CALL(mock_, method).WillOnce(::testing::Return(value));
    }

  }
}
```

**Patterns (C++):**
- Namespace: Tests in `myactuator_rmd::test` namespace
- Fixture-based tests inherit from `::testing::Test`
- Mock setup in `SetUp()` and cleanup in `TearDown()`
- Use of `const` for test objects
- Direct initialization of message objects: `{{0xB2, 0x00, 0x00, 0x00, ...}}`

**Suite Organization (Python):**
```python
@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
```

**Patterns (Python):**
- Pytest markers for categorization (`@pytest.mark.flake8`, `@pytest.mark.linter`)
- Ament integration: uses `main_with_errors()` from ament tools
- Assertions via standard Python `assert` statements

## Mocking

**Framework:** C++ uses Google Mock (GMock)

**Patterns (from** `mock/actuator_actuator_mock_test.hpp`):
```cpp
// Test fixture setup
class ActuatorActuatorMockTest: public ::testing::Test {
public:
    void SetUp() override;     // Starts mock in separate thread
    void TearDown() override;  // Joins mock thread

protected:
    myactuator_rmd::CanDriver driver_;
    myactuator_rmd::ActuatorInterface actuator_;
    ActuatorMock actuator_mock_;
    std::thread mock_thread_;
};

// Mock usage in test
TEST_F(ActuatorActuatorMockTest, getVersionDate) {
    myactuator_rmd::GetVersionDateResponse const response {{0xB2, 0x00, ...}};
    EXPECT_CALL(actuator_mock_, getVersionDate)
        .WillOnce(::testing::Return(response));
    auto const version {actuator_.getVersionDate()};
    EXPECT_EQ(version, 20220206);
}
```

**What to Mock:**
- CAN bus communication (via virtual loopback interface `vcan_test`)
- Motor responses (using `ActuatorMock` class)
- Low-level protocol handling

**What NOT to Mock:**
- Core protocol encoding/decoding (test with real data)
- Configuration dataclass logic
- Unit conversions (test with concrete values)
- Validation logic in dataclass `__post_init__()`

## Fixtures and Factories

**Test Data:**
- Hardcoded CAN message bytes: `{{0xB2, 0x00, 0x00, 0x00, 0x2E, 0x89, 0x34, 0x01}}`
- Configuration objects created directly in tests
- Mock motor responses as Response objects with known data

**Location:**
- C++ mock: `myactuator_rmd/test/mock/`
  - `ActuatorMock` class provides motor behavior simulation
  - Runs in separate thread to simulate real hardware
- Python fixtures: Not used; tests use direct class instantiation

**Example fixture (C++):**
```cpp
class ActuatorActuatorMockTest: public ::testing::Test {
  // Mock runs in separate thread via SetUp()
  // Actuator communicates with mock over virtual CAN
};
```

## Coverage

**Requirements:** C++ tests have coverage tracking enabled
- Enabled via CMake: `cmake .. -DENABLE_COVERAGE=on`
- Measured with `gcovr` (GNU coverage tool)
- Filters coverage to `include/` and `src/` directories only
- Coverage report uploaded to Codecov

**View Coverage:**
```bash
# After running tests with ENABLE_COVERAGE=on
gcovr -j $(nproc) --filter include --filter src --print-summary --xml coverage.xml
# Coverage summary printed to stdout
# XML report written to coverage.xml for CI integration
```

**Python Coverage:** No explicit coverage tracking; ROS 2 linting tests verify style/documentation only

## Test Types

**Unit Tests (C++):**
- Scope: Individual protocol message parsing, CAN utilities, configuration logic
- Approach: Direct testing of encode/decode functions and class methods
- Example: `requests_test.cpp` tests each request type parsing with hardcoded bytes
- Isolation: Uses mock objects; virtual CAN interface for communication tests

**Integration Tests (C++):**
- Scope: ActuatorInterface communicating with mock motor
- Approach: Fixture-based (`TEST_F`) with setup/teardown; mock runs in thread
- Example: `actuator_test.cpp` tests motor commands end-to-end
- Isolation: Virtual CAN loopback (`vcan_test` interface)

**Linting/Style Tests (Python):**
- Scope: Code style (flake8), docstring format (pep257), copyright headers
- Framework: ament_flake8, ament_pep257
- Location: `test_flake8.py`, `test_pep257.py` in URDF package
- Run via: `colcon test`

**E2E Tests:**
- Not found in codebase
- Manual testing required for full system (ROS 2 driver + real motors)
- Integration testing via `ros2 launch` and ROS topic monitoring

## Common Patterns

**Async Testing:**
- C++ uses threading: Mock runs in `std::thread` during `SetUp()`, joined in `TearDown()`
- Simulates real async hardware responses
- No explicit async/await pattern (C++17 standard threading)

**Error Testing:**
- C++: Protocol validation tests check reject/accept of malformed messages
- Example: `SetCanBaudRateRequest` with different payload bytes tests parsing
- Python: Dataclass `__post_init__` raises `ValueError` for invalid configs (testable via pytest)

**Example error validation (Python):**
```python
@dataclass
class HysteresisTorqueTrigger:
    def __post_init__(self):
        if self.direction == "rising":
            if self.exit_threshold_rad >= self.enter_threshold_rad:
                raise ValueError("For rising: exit_threshold must be < enter_threshold")
```

**CAN Interface Setup (Tests):**
```bash
# Required for loopback tests
sudo modprobe vcan
sudo ip link add dev vcan_test type vcan
sudo ip link set up vcan_test
candump vcan_test              # Monitor CAN messages (debugging)
```

**Test Timeout:**
- C++ tests have 10-second timeout (set in CMakeLists.txt)
  ```cmake
  set_tests_properties(${noArgsTests} PROPERTIES TIMEOUT 10)
  ```

## CI/CD Integration

**GitHub Actions:**
- Workflow: `.github/workflows/run-tests.yml` (in myactuator_rmd)
- Triggers: Push to paths affecting tests, monthly schedule, manual dispatch
- Steps:
  1. Install dependencies (cmake, GTest, pybind11, linux-modules for vcan)
  2. Run CMake with testing enabled: `-DBUILD_TESTING=on -DENABLE_COVERAGE=on`
  3. Compile: `make -j$(nproc)`
  4. Set up virtual CAN: `vcan_test` interface
  5. Run tests: `ctest`
  6. Generate coverage: `gcovr`
  7. Upload to Codecov: `codecov-action@v4`
  8. Cleanup: Remove vcan_test interface

---

*Testing analysis: 2026-02-09*
