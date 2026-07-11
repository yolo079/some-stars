// tools/mawesome/index.js
const fs = require('fs');
const path = require('path');
const ejs = require('ejs');
const { Octokit } = require('@octokit/rest');
const argv = require('minimist')(process.argv.slice(2));

async function main() {
  const templatePath = argv.template || argv['template-path'] || './template/README.ejs';
  const githubName = argv['github-name'] || process.env.GITHUB_REPOSITORY_OWNER || process.env.GITHUB_ACTOR;
  const outPath = argv.out || argv.output || 'README.md';
  const token = process.env.PERSONAL_TOKEN || process.env.GITHUB_TOKEN;

  if (!githubName) {
    console.error('Missing --github-name or GITHUB_REPOSITORY_OWNER');
    process.exit(2);
  }

  const octokit = token ? new Octokit({ auth: token }) : new Octokit();

  // Fetch starred repos of the given user (example)
  let stars = [];
  try {
    const per_page = 100;
    let page = 1;
    while (true) {
      const res = await octokit.activity.listReposStarredByUser({
        username: githubName,
        per_page,
        page
      });
      if (!res || !res.data) break;
      stars = stars.concat(res.data);
      if (res.data.length < per_page) break;
      page++;
    }
  } catch (err) {
    console.error('Error fetching starred repos:', err && err.message ? err.message : err);
    // If octokit returns structured error, include status / response data
    if (err.status) console.error('Status:', err.status);
    if (err.response && err.response.data) {
      try {
        console.error('Response data:', JSON.stringify(err.response.data));
      } catch (e) {
        console.error('Response data (raw):', err.response.data);
      }
    }
    process.exit(3);
  }

  // Resolve template path and ensure it exists
  const tplFullPath = path.resolve(templatePath);
  console.log('Using template:', tplFullPath);
  if (!fs.existsSync(tplFullPath)) {
    console.error('Template not found at', tplFullPath);
    process.exit(4);
  }

  // Prepare data for template
  const data = {
    githubName,
    stars: stars.map(r => ({
      name: r.full_name,
      url: r.html_url,
      description: r.description,
      language: r.language,
      stargazers_count: r.stargazers_count,
      owner: r.owner && r.owner.login
    }))
  };

  // Render (use renderFile so EJS can resolve includes; enable async rendering and await it)
  let out;
  try {
    out = await ejs.renderFile(tplFullPath, data, { async: true, filename: tplFullPath });
  } catch (err) {
    console.error('Error rendering template:', err && err.message ? err.message : err);
    if (err && err.stack) console.error(err.stack);
    // If EJS error has filename/line info, log it
    if (err && err.filename) console.error('EJS filename:', err.filename);
    process.exit(5);
  }

  try {
    fs.writeFileSync(outPath, out, 'utf8');
    console.log('Wrote', outPath);
  } catch (err) {
    console.error('Failed to write output file:', err && err.message ? err.message : err);
    process.exit(6);
  }
}

main().catch(err => {
  console.error('Unhandled error:', err && err.message ? err.message : err);
  if (err && err.stack) console.error(err.stack);
  process.exit(1);
});
