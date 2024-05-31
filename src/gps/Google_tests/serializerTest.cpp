#include "gtest/gtest.h"
#include "serdes/UbxSerializer.hpp"

TEST(UbxSerializerTest, givenSerializerObjectIsInstantiated_whenDefaultConstructorIsUsed_thenPayloadIsEmpty)
{
    UbxSerializer serializer;
    EXPECT_EQ(serializer.getPayloadReadOnly().size(), 0);
}

TEST(UbxSerializerTest, givenSerializerObjectIsInstantiated_whenSizeIsGiven_thenPayloadIsCorrectSize)
{
    UbxSerializer serializer(10);
    EXPECT_EQ(serializer.getPayloadReadOnly().capacity(), 10);
}

TEST(UbxSerializerTest, givenEmptyPayload_whenWriteU1_thenPayloadContainsIntBetween0And255)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    const int indicesUsedPerWrite = 1;

    //Act
    serializer.writeU1(0x12); // 0x12 = 18
    serializer.writeU1(1);
    serializer.writeU1(-1);

    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 3);
    EXPECT_EQ(payload[0], 18);
    EXPECT_EQ(payload[1], 1);
    EXPECT_EQ(payload[2], 255);
}

TEST(UbxSerializerTest, givenEmptyPayload_whenWriteU2_thenPayloadContainsIntBetween0And65535)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    const int indicesUsedPerWrite = 2;

    //Act
    serializer.writeU2(0x11); // 0x11 = 17
    serializer.writeU2(1);
    serializer.writeU2(-1); // -1 -> 65535

    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 3);
    EXPECT_EQ(payload[0], 17);
    EXPECT_EQ(payload[1], 0);
    EXPECT_EQ(payload[2], 1);
    EXPECT_EQ(payload[3], 0);
    EXPECT_EQ(payload[4], 255);
    EXPECT_EQ(payload[5], 255);
}

TEST(UbxSerializerTest, givenEmptyPayload_whenWriteU4_thenPayloadContainsIntBetween0And65535)
{
    // Arrange
    UbxSerializer serializer;
    const auto& payload = serializer.getPayloadReadOnly();
    const int indicesUsedPerWrite = 4;

    //Act
    serializer.writeU4(0x24); // 0x24 = 36
    // serializer.writeU4(-1); // -1 -> 2,147,483,647
    serializer.writeU4(0x7FFFFFFF); // = 2,147,483,647


    // Assert
    ASSERT_EQ(payload.size(), indicesUsedPerWrite * 2);
    EXPECT_EQ(payload[0], 36);
    EXPECT_EQ(payload[1], 0);
    EXPECT_EQ(payload[2], 0);
    EXPECT_EQ(payload[3], 0);

    EXPECT_EQ(payload[4], 255);
    EXPECT_EQ(payload[5], 255);
    EXPECT_EQ(payload[6], 255);
    EXPECT_EQ(payload[7], 255);
}
