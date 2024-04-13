import React from "react";

import Link from "@docusaurus/Link";

function Topic({ title, to, children }) {
  return (
    <div class="card">
      <div class="card__header">
        <h3>{title}</h3>
      </div>
      <div class="card__body">
        <p>{children}</p>
      </div>
      <div class="card__footer">
        <Link to={to}>
          <button class="button button--secondary button--block">
            See {title}
          </button>
        </Link>
      </div>
    </div>
  );
}

export default Topic;
