#pragma once

/**
 * @brief Serializes a struct into char array.
 * 
 * @param structure The struct to serialize.
 * 
 * @return The serialized struct.
 */
template<typename T>
char* serialize(const T& structure);

/**
 * @brief Deserializes a char array into given struct.
 * 
 * @param buffer The char array to deserialize.
 * 
 * @return The deserialized struct.
 */
template<typename T>
T deserialize(char* buffer);