const status = document.getElementById("status");
const notice = document.getElementById("notice");

import("/godot_live.js").catch(error => {
  status.textContent = "Desktop viewer could not load";
  notice.hidden = false;
  notice.textContent = `${error.message || error}. Try Reconnect or Standard desktop.`;
  document.getElementById("reconnect").addEventListener("click", () => location.reload());
});
