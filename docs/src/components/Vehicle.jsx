import React from "react";

function Vehicle({ name, img, children }) {
  return (
    <div class="card">
      <div class="card__image">
        <img
          src={img}
          alt="Image alt text"
          title="Logo Title Text 1" />
      </div>
      <div class="card__body">
        <h4>{name}</h4>
        <small>
          {children}
        </small>
      </div>
    </div>
  );
}

export default Vehicle;
