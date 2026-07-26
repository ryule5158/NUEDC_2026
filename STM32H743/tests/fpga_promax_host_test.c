#include "fpga_promax.h"
#include "fpga_promax_link_map.h"
#include <stdio.h>
#include <string.h>

#define TEST_CHECK(condition)                                      \
  do                                                               \
  {                                                                \
    if (!(condition))                                              \
    {                                                              \
      printf("TEST_FAIL line=%d: %s\n", __LINE__, #condition);    \
      return 1;                                                     \
    }                                                              \
  } while (0) /* 主机单测断言宏。 */

/* 模拟FPGA物理寄存器空间。 */
static uint32_t s_registers[128];

/* 模拟响应故障模式。 */
typedef enum
{
  MOCK_RESPONSE_NORMAL = 0, /* 返回正常响应。 */
  MOCK_RESPONSE_BAD_CRC,    /* 返回错误CRC。 */
  MOCK_RESPONSE_BAD_SEQ     /* 返回错误序号。 */
} MockResponseMode;

static MockResponseMode s_response_mode; /* 下一逻辑响应的故障模式。 */

/* 从大端字节数组读取32位整数。 */
static uint32_t TestLoadBe32(const uint8_t *src)
{
  return ((uint32_t)src[0] << 24) |
         ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) |
         (uint32_t)src[3];
}

/* 将32位整数按大端序写入字节数组。 */
static void TestStoreBe32(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 24);
  dst[1] = (uint8_t)(value >> 16);
  dst[2] = (uint8_t)(value >> 8);
  dst[3] = (uint8_t)value;
}

/* 组装模拟FPGA逻辑响应。 */
static void TestBuildResponse(const uint8_t *request,
                              uint8_t status,
                              uint32_t value,
                              uint8_t *response)
{
  response[0] = FPGA_PROMAX_RESPONSE_SOF;
  response[1] = request[1];
  response[2] = status;
  response[3] = request[3];
  TestStoreBe32(&response[4], value);
  response[8] = FpgaPromax_Crc8(response, 8U);

  if (s_response_mode == MOCK_RESPONSE_BAD_CRC)
  {
    response[8] ^= 1U;
  }
  else if (s_response_mode == MOCK_RESPONSE_BAD_SEQ)
  {
    response[1]++;
    response[8] = FpgaPromax_Crc8(response, 8U);
  }
  else
  {
    /* 正常响应无需修改。 */
  }
  s_response_mode = MOCK_RESPONSE_NORMAL;
}

/* 用生产地址映射模拟一轮ProMax逻辑事务。 */
static int32_t TestMockTransport(
  void *user,
  const uint8_t request[FPGA_PROMAX_FRAME_SIZE],
  uint8_t response[FPGA_PROMAX_FRAME_SIZE],
  uint32_t timeout_ms)
{
  uint32_t value;
  uint8_t command;
  uint8_t logical;
  uint8_t physical;
  uint8_t status;

  (void)user;
  if ((request == NULL) || (response == NULL) || (timeout_ms == 0U) ||
      (request[0] != FPGA_PROMAX_REQUEST_SOF))
  {
    return -1;
  }
  if (FpgaPromax_Crc8(request, 8U) != request[8])
  {
    TestBuildResponse(request, FPGA_PROMAX_STATUS_CRC, 0U, response);
    return 0;
  }

  command = request[2];
  logical = request[3];
  value = 0U;
  status = FPGA_PROMAX_STATUS_OK;
  if (command == FPGA_PROMAX_CMD_PING)
  {
    (void)FPGA_ProMaxMapAddress(FPGA_PROMAX_REG_ID, &physical);
    value = s_registers[physical];
  }
  else if ((command != FPGA_PROMAX_CMD_READ) &&
           (command != FPGA_PROMAX_CMD_WRITE))
  {
    status = FPGA_PROMAX_STATUS_BAD_CMD;
  }
  else if (FPGA_ProMaxMapAddress(logical, &physical) == 0U)
  {
    status = FPGA_PROMAX_STATUS_BAD_ADDR;
  }
  else if ((command == FPGA_PROMAX_CMD_WRITE) &&
           (FPGA_ProMaxIsWritable(logical) == 0U))
  {
    status = FPGA_PROMAX_STATUS_READ_ONLY;
  }
  else if (command == FPGA_PROMAX_CMD_WRITE)
  {
    value = TestLoadBe32(&request[4]);
    s_registers[physical] = value;
    value = 0U;
  }
  else
  {
    value = s_registers[physical];
  }

  TestBuildResponse(request, status, value, response);
  return 0;
}

/* 校验全部逻辑地址映射边界和写属性。 */
static int TestAddressMap(void)
{
  static const uint8_t logical[] = {
    0x00U, 0x05U, 0x10U, 0x17U, 0x20U, 0x27U,
    0x40U, 0x41U, 0x50U, 0x57U, 0x58U, 0x5FU,
    0x60U, 0x67U, 0x70U, 0x7AU
  }; /* 合法逻辑地址边界。 */
  static const uint8_t physical[] = {
    0x30U, 0x35U, 0x38U, 0x3FU, 0x40U, 0x47U,
    0x48U, 0x49U, 0x50U, 0x57U, 0x58U, 0x5FU,
    0x60U, 0x67U, 0x68U, 0x72U
  }; /* 对应物理地址边界。 */
  static const uint8_t invalid[] = {
    0x06U, 0x0FU, 0x18U, 0x28U, 0x3FU, 0x42U,
    0x4FU, 0x68U, 0x80U, 0x85U
  }; /* 当前实时档未映射逻辑地址。 */
  uint8_t mapped;
  size_t index;

  for (index = 0U; index < (sizeof(logical) / sizeof(logical[0])); index++)
  {
    TEST_CHECK(FPGA_ProMaxMapAddress(logical[index], &mapped) == 1U);
    TEST_CHECK(mapped == physical[index]);
  }
  for (index = 0U; index < (sizeof(invalid) / sizeof(invalid[0])); index++)
  {
    TEST_CHECK(FPGA_ProMaxMapAddress(invalid[index], &mapped) == 0U);
  }
  TEST_CHECK(FPGA_ProMaxMapAddress(0U, NULL) == 0U);

  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_CONTROL) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_SNAPSHOT) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(0x10U) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(0x27U) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_MF_COEFF) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_WAVE_CONTROL) == 1U);
  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_ID) == 0U);
  TEST_CHECK(FPGA_ProMaxIsWritable(FPGA_PROMAX_REG_POWER_LO0) == 0U);

  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_CONTROL,
                                    0x00000003U,
                                    0x00000001U) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_CONTROL,
                                    0x00000001U,
                                    0x00000000U) == 0U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_SNAPSHOT,
                                    1U,
                                    0xA5A50001U) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_PHASE_INC0,
                                    0x12345678U,
                                    0x12345678U) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_PHASE_INC0,
                                    0x12345678U,
                                    0x12345679U) == 0U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_MF_SELECTOR,
                                    0xFFFFFFFFU,
                                    0x0000031FU) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_MF_COEFF,
                                    0xFFFFFFFFU,
                                    0x0003FFFFU) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_WAVE_CONTROL,
                                    0x00000106U,
                                    0x00000006U) == 1U);
  TEST_CHECK(FPGA_ProMaxVerifyWrite(FPGA_PROMAX_REG_STATUS,
                                    0U,
                                    0U) == 0U);
  return 0;
}

/* 校验通用API、能力门控、快照和错误检测。 */
static int TestCoreApi(void)
{
  FpgaPromax dev;
  FpgaPromax_AmEstimate am;
  FpgaPromax_FftPeak fft_peak;
  FpgaPromax_WaveMetrics metrics;
  FpgaPromax_WaveStats wave;
  FpgaPromax_Result result;
  int32_t score;
  uint32_t index;
  uint32_t value;
  uint64_t power;
  int32_t fir_coeff[FPGA_PROMAX_MF_TAPS];
  size_t tap;

  memset(s_registers, 0, sizeof(s_registers));
  s_registers[0x30U] = FPGA_PROMAX_EXPECTED_ID;
  s_registers[0x31U] = 0x00010100U;
  s_registers[0x32U] = FPGA_PROMAX_CAPABILITY_REALTIME |
                       FPGA_PROMAX_CAPABILITY_WAVE_STATS;
  s_registers[0x34U] = FPGA_PROMAX_STATUS_RUN |
                       FPGA_PROMAX_STATUS_ALL_RESULTS;

  result = FpgaPromax_Init(&dev, TestMockTransport, NULL, 10U);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK(dev.version == 0x00010100U);
  TEST_CHECK(FpgaPromax_HasRealtime(&dev) == 1U);
  TEST_CHECK(FpgaPromax_HasFft(&dev) == 0U);
  TEST_CHECK(FpgaPromax_HasWaveStats(&dev) == 1U);
  TEST_CHECK(FpgaPromax_SetFft(&dev, 1U, 0U, 1U) ==
             FPGA_PROMAX_E_UNSUPPORTED);

  result = FpgaPromax_SetDdcFrequency(&dev, 0U, 1000000U, 32000000U);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK(s_registers[0x38U] == 0x08000000U);
  TEST_CHECK(s_registers[0x40U] == 0U);
  TEST_CHECK(FpgaPromax_SetDdcFrequency(&dev, 0U, 16000001U, 32000000U) ==
             FPGA_PROMAX_E_RANGE);

  for (tap = 0U; tap < FPGA_PROMAX_MF_TAPS; tap++)
  {
    fir_coeff[tap] = 0;
  }
  fir_coeff[FPGA_PROMAX_MF_TAPS - 1U] = 65536;
  TEST_CHECK(FpgaPromax_LoadFirBank(&dev, 2U, fir_coeff,
                                    FPGA_PROMAX_MF_TAPS) ==
             FPGA_PROMAX_OK);
  TEST_CHECK(s_registers[0x48U] == 0x0000021FU);
  TEST_CHECK(s_registers[0x49U] == 0x00010000U);
  fir_coeff[0] = 32768;
  TEST_CHECK(FpgaPromax_LoadCorrelationTemplate(
               &dev, 3U, fir_coeff, FPGA_PROMAX_MF_TAPS) == FPGA_PROMAX_OK);
  TEST_CHECK(s_registers[0x48U] == 0x0000031FU);
  TEST_CHECK(s_registers[0x49U] == 0x00008000U);

  result = FpgaPromax_SetWaveBlockLog2(&dev, 6U);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK((s_registers[0x68U] & 0x1FU) == 6U);
  s_registers[0x69U] = 0x00060203U;
  s_registers[0x6AU] = 0x0000649CU;
  s_registers[0x6BU] = 0U;
  s_registers[0x6CU] = 0U;
  s_registers[0x6DU] = 321312U;
  s_registers[0x6EU] = 0U;
  s_registers[0x6FU] = 32U;
  s_registers[0x70U] = 8U;
  s_registers[0x71U] = 56U;
  s_registers[0x72U] = 64U;
  result = FpgaPromax_ReadWaveStats(&dev, &wave, 10U);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK((wave.minimum_code == -100) &&
             (wave.maximum_code == 100) &&
             (wave.sample_count == 64U) &&
             (wave.generation == 2U));
  result = FpgaPromax_ComputeWaveMetrics(&wave,
                                         32000000U,
                                         0.0f,
                                         1.0f,
                                         &metrics);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK((metrics.vpp > 199.9f) && (metrics.vpp < 200.1f));
  TEST_CHECK((metrics.rms > 70.8f) && (metrics.rms < 70.9f));
  TEST_CHECK((metrics.frequency_hz > 3999999.0f) &&
             (metrics.frequency_hz < 4000001.0f));
  TEST_CHECK((metrics.duty_percent > 49.9f) &&
             (metrics.duty_percent < 50.1f));

  {
    static const uint32_t rf_tones[3] = {
      35000000U, 37000000U, 39000000U
    };
    result = FpgaPromax_ConfigureToneBank(&dev, rf_tones, 3U, 32000000U);
    TEST_CHECK(result == FPGA_PROMAX_OK);
    TEST_CHECK(s_registers[0x38U] == 0x18000000U);
  }

  s_registers[0x50U] = 0x89ABCDEFU;
  s_registers[0x58U] = 0x0000001FU;
  result = FpgaPromax_GetBandPower(&dev, 0U, &power);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK(power == 0x0000001F89ABCDEFULL);
  TEST_CHECK((s_registers[0x35U] & 1U) == 0U);

  s_registers[0x60U] = 0x00FFFFFFU;
  s_registers[0x64U] = 1234U;
  result = FpgaPromax_GetFirPeak(&dev, 0U, &score, &index);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK(score == -1);
  TEST_CHECK(index == 1234U);

  s_registers[0x50U] = 1000000U;
  s_registers[0x58U] = 0U;
  s_registers[0x51U] = 62500U;
  s_registers[0x59U] = 0U;
  s_registers[0x52U] = 62500U;
  s_registers[0x5AU] = 0U;
  result = FpgaPromax_ReadAmEstimate(&dev, &am, 10U);
  TEST_CHECK(result == FPGA_PROMAX_OK);
  TEST_CHECK((am.modulation_depth > 0.499f) &&
             (am.modulation_depth < 0.501f));

  result = FpgaPromax_WriteRegister(&dev, FPGA_PROMAX_REG_ID, 1U);
  TEST_CHECK(result == FPGA_PROMAX_E_FPGA_STATUS);
  TEST_CHECK(dev.last_protocol_status == FPGA_PROMAX_STATUS_READ_ONLY);
  result = FpgaPromax_ReadRegister(&dev, 0x80U, &value);
  TEST_CHECK(result == FPGA_PROMAX_E_FPGA_STATUS);
  TEST_CHECK(dev.last_protocol_status == FPGA_PROMAX_STATUS_BAD_ADDR);

  s_response_mode = MOCK_RESPONSE_BAD_CRC;
  TEST_CHECK(FpgaPromax_GetStatus(&dev, &value) == FPGA_PROMAX_E_CRC);
  s_response_mode = MOCK_RESPONSE_BAD_SEQ;
  TEST_CHECK(FpgaPromax_GetStatus(&dev, &value) == FPGA_PROMAX_E_SEQUENCE);
  TEST_CHECK(FpgaPromax_GetFftPeak(&dev, &fft_peak, 10U) ==
             FPGA_PROMAX_E_UNSUPPORTED);
  return 0;
}

/* 执行全部不依赖实板的ProMax主机测试。 */
int main(void)
{
  static const uint8_t crc_text[] = "123456789"; /* CRC标准检查字符串。 */

  TEST_CHECK(FpgaPromax_Crc8(crc_text, sizeof(crc_text) - 1U) == 0xF4U);
  TEST_CHECK(FpgaPromax_Crc8(NULL, 1U) == 0U);
  TEST_CHECK(TestAddressMap() == 0);
  TEST_CHECK(TestCoreApi() == 0);
  printf("FPGA_PROMAX_HOST_TEST_PASS\n");
  return 0;
}
