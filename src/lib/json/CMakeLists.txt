include(FetchContent)

# Fetch nlohmann::json. Version 3.12.0 still has some bugs with std::optional.
FetchContent_Declare(json URL https://github.com/nlohmann/json/archive/3ed64e502a6371311af3c2f309e6525b2f5f6f18.tar.gz)
FetchContent_MakeAvailable(json)
