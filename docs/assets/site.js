// Theme toggle and the judged-run dots. The page reads fine without this file.
(function () {
  var root = document.documentElement;
  root.classList.add("js");

  // Theme: remembered choice first, otherwise the system setting.
  var saved = null;
  try { saved = localStorage.getItem("plugbot-theme"); } catch (e) {}
  if (saved === "light" || saved === "dark") root.setAttribute("data-theme", saved);

  function current() {
    var t = root.getAttribute("data-theme");
    if (t) return t;
    return window.matchMedia && matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
  }

  document.addEventListener("DOMContentLoaded", function () {
    var btn = document.querySelector(".theme");
    if (btn) {
      var sync = function () { btn.setAttribute("aria-pressed", current() === "dark" ? "true" : "false"); };
      sync();
      btn.addEventListener("click", function () {
        var next = current() === "dark" ? "light" : "dark";
        root.setAttribute("data-theme", next);
        try { localStorage.setItem("plugbot-theme", next); } catch (e) {}
        sync();
      });
    }

    // <div class="dots" data-runs="ok:19,no:1"> becomes one dot per run, in the order given.
    var i = 0;
    document.querySelectorAll(".dots[data-runs]").forEach(function (box) {
      box.getAttribute("data-runs").split(",").forEach(function (part) {
        var kv = part.split(":"), cls = kv[0], n = parseInt(kv[1], 10);
        for (var k = 0; k < n; k++) {
          var d = document.createElement("span");
          d.className = "dot " + cls;
          d.style.setProperty("--i", i++);
          box.appendChild(d);
        }
      });
    });
  });
})();
