import React from "react";

function Vehicle({ name, img, children }) {
  return (
    <div className="card">
      <div className="card__image">
        <img
          src={img}
          alt="Image alt text"
          title="Logo Title Text 1" />
      </div>
      <div className="card__body">
        <h4>{name}</h4>
        <small>
          {children}
        </small>
      </div>
    </div>
  );
}

export default Vehicle;
