if (process.env.NODE_ENV==='development') {
  // Must use require here as import statements are only allowed
  // to exist at top-level.
  require("preact/debug");
}
//import {render} from 'react-dom'
import { render } from 'preact';
import { h } from 'preact';
import React from 'react';
import App from './App';
import "babel-polyfill";
import ReactDOM from 'react-dom';

ReactDOM.render(<App />, document.getElementById('root'));

