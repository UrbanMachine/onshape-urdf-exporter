
Installation & requirements
===========================

Requirements
-------------

You will need an Onshape account and Python 3.

Installation
------------

Pipx_ is the recommended way to install Onshape URDF Exporter. Just run the
following command:

.. code-block:: bash

    pipx install onshape-urdf-exporter

.. _Pipx: https://pipx.pypa.io/stable/installation/

.. _api-key:

Setting up your API key
-----------------------

To authenticate the tool with Onshape, you will need to obtain an API key and
secret from the `Onshape developer portal`_.

These keys are provided to the tool using environment variables. Declare them
in your ``.bashrc`` (or shell equivalent) by adding these lines:

.. code-block:: bash

    # Obtained at https://dev-portal.onshape.com/keys
    export ONSHAPE_API=https://cad.onshape.com
    export ONSHAPE_ACCESS_KEY=Your_Access_Key
    export ONSHAPE_SECRET_KEY=Your_Secret_Key

.. _Onshape developer portal: https://dev-portal.onshape.com/keys
