# Embeds

This folder contains the HTML JavaScript embeds for the **interactive 3D models** and **circuit design**.

The interactive models are presented in web pages and allow users to view and manipulate 3D objects directly in their browser.
We opted to include this feature primarily to address the limitations of GitHub's .stl file visualization, which lacks color support and offers restricted viewing capabilities.

On the other hand, the interactive circuit was designed using the **Cirkit Designer App**. This platform is ideal for understanding the component connections to the Arduino MEGA 2560 Embed, especially with its built-in chatbot and pin visualization features.

---

## Visualize the different interactive components

### [Interactive Circuit Design](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_circuit)

### [Interactive Chassis](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_chassis)

### [Interactive Hub](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_hub)

### [Interactive Rear Wheels](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_rear_wheels)

### [Interactive Rims](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_rims)

### [Interactive Steering Rod](https://vizdrive.github.io/VizDrive_WRO2025/embeds/interactive_steering_rods)

## How These Embeds Work

## Interactive 3D Models

The HTML codes creates a simple web page that displays an **interactive 3D model**. It uses the `<model-viewer>` web component, the tool that makes it easy to embed 3D models on the web with capabilities like augmented reality (AR) and user controls.

**Example:** interactive_chassis.html

### HTML Structure (`<!DOCTYPE html>`, `<html>`, `<head>`, `<body>`)

* **`<!DOCTYPE html>` and `<html lang="es">`**: Standard declarations defining the document type as HTML5 and setting the language to Spanish.
* **`<head>`**: Contains metadata about the page:
    * **`<meta charset="UTF-8">`**: Specifies the character encoding, ensuring text displays correctly.
    * **`<meta name="viewport" content="width=device-width, initial-scale=1.0">`**: Configures the viewport for responsive design, making the page look good on various devices.
    * **`<title>Car Chassis Interactive 3D Model</title>`**: Sets the title that appears in the browser tab.
    * **`<script type="module" src="https://ajax.googleapis.com/ajax/libs/model-viewer/3.4.0/model-viewer.min.js"></script>`**: This is the most crucial line for the 3D model. It imports the **`model-viewer`** library, a web component developed by Google. This script enables the custom `<model-viewer>` HTML tag to display 3D models. The `type="module"` ensures it's loaded as an ES module.
    * **`<style>`**: Contains **CSS (Cascading Style Sheets)** rules that define the visual appearance of the web page. This includes:
        * Centering content on the page (`body`).
        * Styling the headings (`h1`).
        * Defining the size, background, border, and shadow for the `<model-viewer>` element itself.
        * Styling for the `controls` section and links (`a`).

### Body Content (`<body>`)

* **`<h1>Interactive 3D Model</h1>` and `<h1>Car Chassis</h1>`**: These are headings that display the title of the interactive model on the web page.
* **`<model-viewer>` Tag**: It's a custom HTML element provided by the `model-viewer` library.
    * **`src="./models_glb/chassis.glb"`**: This attribute points to the 3D model file itself. The `.glb` format is a binary version of glTF, optimized for efficient loading and transmission.
    * **`alt="Chassis 3D Model"`**: Provides alternative text for accessibility, describing the content for screen readers.
    * **`ar`**: Enables **Augmented Reality (AR)** functionality. If the user's device supports it, they can view the 3D model placed in their real-world environment using their camera.
    * **`shadow-intensity="1"`**: Controls the strength of the shadow cast by the 3D model.
    * **`camera-controls`**: This is essential for user interaction. It allows users to **rotate, pan, and zoom** the 3D model using their mouse or touch gestures.
    * **`touch-action="pan-y"`**: Specifies how touch gestures are handled. `pan-y` allows vertical panning while reserving horizontal movement for model rotation, which can improve user experience on touch devices.
    * **`autoplay`**: Makes the model automatically animate or rotate when it loads.
    * **`exposure="1.2"`**: Adjusts the brightness of the model's lighting.
    * **`environment-image="neutral"`**: Sets the ambient lighting environment for the model. "neutral" provides a balanced lighting setup.
    * **`bounds="tight"`**: Causes the camera to tightly frame the model.
    * **`tone-mapping="neutral"`**: Controls how colors are rendered, aiming for a natural look.
    * **`loading="eager"`**: Instructs the browser to load the model as soon as possible.
    * **`<div slot="poster" class="poster"></div>` and `<div slot="progress-bar" class="progress-bar"></div>`**: These are optional slots for custom elements that `model-viewer` can display. The `poster` slot is typically used for an image shown before the model loads, and the `progress-bar` slot is for a loading indicator. They are empty, so they won't visibly do anything unless styled with CSS or filled with content.

* **`<div class="controls">`**: This section provides **user instructions** for interacting with the 3D model.
    * It explains how to rotate (drag with mouse/finger) and how to zoom (mouse wheel/pinch).
  
* **`<p><a href="javascript:history.back()">Go back to main page</a></p>`**: A simple paragraph with a link that, when clicked, uses JavaScript (`history.back()`) to navigate the user to the previous page in their browser history.

## Interactive Circuit Design

This HTML code creates a simple web page designed to **embed an interactive circuit diagram** hosted on the Cirkit Designer platform.

### HTML Structure 

It's a standard HTML5 page with a basic `head` (containing metadata and styling) and a `body` (with the visible content).

* **Styling (`<style>` tag)**: The CSS here primarily focuses on making the embedded content **responsive**.
    * `embed-container`: This class uses a common trick for responsive embeds. By setting `position: relative` and using `padding-top` based on `width`, it creates an aspect ratio that ensures the `iframe` (which is `position: absolute` inside it) scales correctly while maintaining its proportions. The `calc(max(56.25%, 400px))` ensures the height is at least 400px or scales with a 16:9 aspect ratio, whichever is larger.
    * `iframe`: Styles the `iframe` to fill its container and removes the default border.
* **Content**:
    * **Headings and Paragraphs**: Provide a title for the page and instructions for the user (e.g., "Click on 'Explore this Circuit' to interact with the model and access the Cirkit chatbot.").
    * **The `<iframe>`**: This is the core of the embed.
        * `src="https://app.cirkitdesigner.com/project/c9dde92c-847c-48cf-9794-15c1a4459869?view=interactive_preview"`: This attribute points to our specific project on the Cirkit Designer platform. The `?view=interactive_preview` parameter tells Cirkit to display the project in an interactive, embeddable preview mode.
    * **Links**:
        * One link allows users to directly **edit the project** in Cirkit Designer (opening in a new tab due to `target="_blank"`).
        * The link, `javascript:history.back()`, allows users to **navigate back** to the previous page in their browser history.
