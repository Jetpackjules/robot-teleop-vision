// Retry independent visual assets without restarting the renderer or robot.
export async function fetchRobotModel(url, {
  signal, decode = value => value, attempts = 3, timeoutMs = 30000,
  retryDelayMs = 500, onRetry = () => {},
} = {}) {
  for (let attempt = 0; attempt < attempts; attempt++) {
    if (signal?.aborted) return null;
    const request = new AbortController();
    const abort = () => request.abort();
    signal?.addEventListener('abort', abort, {once: true});
    let expired = false;
    const timer = setTimeout(() => { expired = true; request.abort(); }, timeoutMs);
    let failure;
    try {
      const response = await fetch(url, {cache: 'no-cache', signal: request.signal});
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      // Keep the deadline active through the body, not only response headers.
      const result = decode(await response.arrayBuffer());
      if (signal?.aborted) return null;
      return result;
    } catch (error) {
      if (signal?.aborted) return null;
      failure = expired ? new Error('model download timed out') : error;
    } finally {
      clearTimeout(timer);
      signal?.removeEventListener('abort', abort);
    }
    if (attempt + 1 === attempts) throw failure;
    onRetry(failure);
    await new Promise(resolve => {
      const finish = () => {
        clearTimeout(delay);
        signal?.removeEventListener('abort', finish);
        resolve();
      };
      const delay = setTimeout(finish, retryDelayMs * 2 ** attempt);
      signal?.addEventListener('abort', finish, {once: true});
      if (signal?.aborted) finish();
    });
  }
}
