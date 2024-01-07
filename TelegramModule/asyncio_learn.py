# First we need the asyncio library
  # stands for Asynchronous Input Output
  # Telegram phải gọi API rất nhiều, nên trong lúc chờ API dùng async sẽ chạy đc code khác mà k bị block
  # Một cách khác: dùng thread, nhưng thread sẽ tốn nhiều bộ nhớ hơn
  # Tốc độ tăng, thay vị để CPU switch qua lại thì code có thể xử lí luôn
  # Chỉ hỗ trợ từ 3.5 trở lên
import asyncio

# Coroutine function of Python
async def main():
    for char in 'Hello, world!\n':
        print(char, end='', flush=True)
        # Với await, hàm cho phép OS chạy code khác trong lúc chờ
        await asyncio.sleep(0.2)

# Then, we can create a new asyncio loop and use it to run our coroutine.
# The creation and tear-down of the loop is hidden away from us.
asyncio.run(main())