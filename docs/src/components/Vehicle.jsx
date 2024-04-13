import React from "react";

function Vehicle({ name, image, children }) {
  return (
    <div class="card">
      <div class="card__image">
        <img
          src="https://images.unsplash.com/photo-1506624183912-c602f4a21ca7?ixlib=rb-1.2.1&ixid=eyJhcHBfaWQiOjEyMDd9&auto=format&fit=crop&w=800&q=60"
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
