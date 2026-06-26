/**
 * HoloLib documentation enhancements.
 *
 * Progressive enhancement only -- the docs are fully readable without this.
 *  1. Prepends a full-height "holographic" title hero to the main page,
 *     populated from the Doxygen project name + brief.
 *  2. Reveals each content block with a gentle fade/slide as it scrolls into
 *     view (skipped entirely under prefers-reduced-motion).
 */
(function () {
    "use strict";

    /* Only the README mainpage (index.html) gets the hero treatment. */
    function isMainPage() {
        var p = location.pathname.replace(/\\/g, "/");
        return p.endsWith("/") || p.endsWith("/index.html") || p === "index.html";
    }

    function text(id, fallback) {
        var el = document.getElementById(id);
        return (el && el.textContent ? el.textContent.trim() : "") || fallback;
    }

    function buildHero(contents, firstBlock) {
        if (document.querySelector(".holo-hero")) return;

        var name = text("projectname", "HoloLib");
        var brief = text("projectbrief", "");

        var hero = document.createElement("section");
        hero.className = "holo-hero";
        hero.innerHTML =
            '<div class="holo-hero__bg" aria-hidden="true"></div>' +
            '<div class="holo-hero__inner">' +
                '<h1 class="holo-hero__title">' + name + "</h1>" +
                '<p class="holo-hero__tagline">' + brief + "</p>" +
                '<a class="holo-hero__scroll" href="#holo-start" ' +
                    'aria-label="Scroll to documentation">' +
                    "<span>Scroll to explore</span>" +
                    '<span class="holo-hero__chevron" aria-hidden="true">&#8595;</span>' +
                "</a>" +
            "</div>";

        contents.insertBefore(hero, contents.firstChild);
        document.documentElement.classList.add("holo-mainpage");

        var target = firstBlock || hero.nextElementSibling;
        if (target) target.id = target.id || "holo-start";

        hero.querySelector(".holo-hero__scroll").addEventListener("click", function (e) {
            e.preventDefault();
            if (target) target.scrollIntoView({ behavior: "smooth", block: "start" });
        });
    }

    function setupReveal(container) {
        if (!("IntersectionObserver" in window)) return;
        if (window.matchMedia("(prefers-reduced-motion: reduce)").matches) return;

        var blocks = container.querySelectorAll(
            ":scope > h1, :scope > h2, :scope > h3, :scope > p, :scope > ul, " +
            ":scope > ol, :scope > table, :scope > div.fragment, :scope > pre, " +
            ":scope > blockquote, :scope > div.image, :scope > dl"
        );
        if (!blocks.length) return;

        var io = new IntersectionObserver(function (entries) {
            entries.forEach(function (entry) {
                if (entry.isIntersecting) {
                    entry.target.classList.add("holo-in");
                    io.unobserve(entry.target);
                }
            });
        }, { threshold: 0.1, rootMargin: "0px 0px -7% 0px" });

        blocks.forEach(function (el) {
            el.classList.add("holo-reveal");
            io.observe(el);
        });
    }

    function init() {
        if (!isMainPage()) return;
        var contents = document.querySelector("div.contents");
        if (!contents) return;

        /* Doxygen wraps the rendered README in div.textblock. */
        var body = contents.querySelector(".textblock") || contents;
        buildHero(contents, body);
        setupReveal(body);
    }

    if (document.readyState === "loading") {
        document.addEventListener("DOMContentLoaded", init);
    } else {
        init();
    }
})();
